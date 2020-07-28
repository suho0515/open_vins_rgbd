/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "Map.h"



using namespace ov_msckf;

double RAD(double deg) { return deg*M_PI / 180.0; }

Map::Map() {
    trackFEATS = new TrackKLT();
}


void Map::set_calibration(std::map<size_t,Eigen::VectorXd> camera_calib)
{
    for (auto const &cam : camera_calib) {
        // Assert we are of size eight
        assert(cam.second.rows()==8);
        // Camera matrix
        cv::Matx33d tempK;
        tempK(0, 0) = cam.second(0);
        tempK(0, 1) = 0;
        tempK(0, 2) = cam.second(2);
        tempK(1, 0) = 0;
        tempK(1, 1) = cam.second(1);
        tempK(1, 2) = cam.second(3);
        tempK(2, 0) = 0;
        tempK(2, 1) = 0;
        tempK(2, 2) = 1;
        camera_k_OPENCV.insert({cam.first, tempK});
        // Distortion parameters
        cv::Vec4d tempD;
        tempD(0) = cam.second(4);
        tempD(1) = cam.second(5);
        tempD(2) = cam.second(6);
        tempD(3) = cam.second(7);
        camera_d_OPENCV.insert({cam.first, tempD});
    }

    return;
}

void Map::feed_depth(double timestamp, cv::Mat &depth_img, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    cv::imshow("depth_img",depth_img);
    // depth image filtering ( bacause there are too many noise )
    cv::Mat median_filtered_img;
    cv::medianBlur(depth_img,median_filtered_img,7);
    cv::imshow("median_filtered_img",median_filtered_img);

    // convert depth image, for using processing
    // raw depth image is "CV_16U", it's not useful for processing
    cv::Mat depth_img_8u;
    double min, max;
    cv::minMaxIdx(median_filtered_img, &min, &max);    
    median_filtered_img.convertTo(depth_img_8u, CV_8U,255.0/(max-min),-255.0*min/(max-min));
    cv::imshow("depth_img_8u",depth_img_8u);

    //trackFEATS->feed_monocular(timestamp, depth_img_8u, 0);

    cv::waitKey(10);
    // // histogram equalization
    // cv::Mat equal_img;
    // cv::equalizeHist( depth_img_8u, equal_img );
    // cv::imshow("equal_img",equal_img);

    // // Extract the new image pyramid
    // std::vector<cv::Mat> imgpyr;
    // cv::buildOpticalFlowPyramid(equal_img, imgpyr, win_size, pyr_levels);

    // if(last_pts.empty()) {
    //     feature_detection(imgpyr, last_pts);
    //     last_depth_img = depth_img;
    //     last_imgpyr = imgpyr;
    //     return;
    // }

    // last_depth_img = depth_img;
    // last_imgpyr = imgpyr;

    // getting point cloud data from depth image
    pointcloud = depth_to_pointcloud(depth_img);

    // filtering pointcloud
    pointcloud = pointcloud_filtering(pointcloud);

    //pointcloud_data.emplace_back(pointcloud);

    //ICP(pointcloud_data);

    

    rT2 =  boost::posix_time::microsec_clock::local_time();
    
    return;
}

void Map::feature_detection(std::vector<cv::Mat> &img0pyr, std::vector<cv::KeyPoint> &pts0) {





    int min_px_dist = 10;
    Eigen::MatrixXi grid_2d = Eigen::MatrixXi::Zero((int)(img0pyr.at(0).rows/min_px_dist)+15, (int)(img0pyr.at(0).cols/min_px_dist)+15);
    auto it0 = pts0.begin();

    while(it0 != pts0.end()) {
        // Get current left keypoint
        cv::KeyPoint kpt = *it0;
        // Check if this keypoint is near another point
        if(grid_2d((int)(kpt.pt.y/min_px_dist),(int)(kpt.pt.x/min_px_dist)) == 1) {
            it0 = pts0.erase(it0);
            continue;
        }
        // Else we are good, move forward to the next point
        grid_2d((int)(kpt.pt.y/min_px_dist),(int)(kpt.pt.x/min_px_dist)) = 1;
        it0++;
    }

    // First compute how many more features we need to extract from this image
    int num_features = 200;
    int num_featsneeded = num_features - (int)pts0.size();

    // If we don't need any features, just return
    if(num_featsneeded < 1)
        return;








    cv::waitKey(10);

}

std::vector<Eigen::Vector3d> Map::depth_to_pointcloud(cv::Mat &depth_img) {

    //float fx = camera_k_OPENCV.at(0)(0,0);
    //float fy = camera_k_OPENCV.at(0)(0,2);
    //float cx = camera_k_OPENCV.at(0)(1,1);
    //float cy = camera_k_OPENCV.at(0)(1,2);

    // depth camera intrinsic
    float fx = 380.70391845703125;
    float fy = 323.19378662109375;
    float cx = 380.70391845703125;
    float cy = 242.1943817138672;



    // 20200715, edited by suho.
    // depth image filtering ( bacause there are too many noise )
    // cv::imshow("depth_img",depth_img);
    // cv::medianBlur(depth_img,depth_img,7);
    //cv::imshow("depthmap_rgb",depthmap_rgb);
    //std::cout<<"Mat.type() : "<<depth_img.type()<<std::endl;
    //std::cout<<"Mat.channels() : "<<depth_img.channels()<<std::endl;

    // let's try median filter to see how it's gonna be stable.
    // it sure has effect.
    // more filtering more stable, but compute cost is increase too.
    // cv::Mat median_filtered_img;
    // cv::medianBlur(depth_img,median_filtered_img,7);
    // cv::medianBlur(median_filtered_img,median_filtered_img,7);
    // cv::medianBlur(median_filtered_img,median_filtered_img,7);
    // cv::medianBlur(median_filtered_img,median_filtered_img,7);
    // cv::medianBlur(median_filtered_img,median_filtered_img,7);
    // cv::imshow("median_filtered_img",median_filtered_img);

    // convert depth image, for using processing
    // raw depth image is "CV_16U", it's not useful for processing
    // cv::Mat depth_img_8u;
    // double min, max;
    // cv::minMaxIdx(median_filtered_img, &min, &max);    
    // median_filtered_img.convertTo(depth_img_8u, CV_8U,255.0/(max-min),-255.0*min/(max-min));
    // cv::imshow("depth_img_8u",depth_img_8u);
    //std::cout<<"depth_img_8u.channels() : "<<depth_img_8u.channels()<<std::endl;
    //std::cout<<"min : "<<min<<std::endl;
    //std::cout<<"max : "<<max<<std::endl;

    // let's multiply some value to depth image to see clear
    //cv::Mat multipled_img;
    //multipled_img = depth_img_8u*8;
    //cv::imshow("multipled_img",multipled_img);



    // try bilateral filter to see what would be better filter
    //cv::Mat bilateral_img;
    //cv::bilateralFilter(depth_img_8u,bilateral_img, 10, 50, 50);
    //cv::imshow("bilateral_img",bilateral_img);

    // interpolation
    //cv::Mat interpolated_img;
    //cv::resize(depth_img_8u,interpolated_img,cv::Size( depth_img_8u.cols, depth_img_8u.rows ), 0, 0, CV_INTER_LINEAR);
    //cv::imshow("interpolated_img",interpolated_img);



    // histogram equalization
    // cv::Mat equal_img;
    // cv::equalizeHist( depth_img_8u, equal_img );
    // cv::imshow("equal_img",equal_img);


    // we need to care about noise of edge.
    // let's denoise it.
    //cv::Mat edge_img;
    //cv::Canny(depth_img_8u,edge_img,50,100);
    //cv::imshow("edge_img",edge_img);

    //cv::Mat edge_img_2;
    //cv::Canny(median_filtered_img,edge_img_2,50,100);
    //cv::imshow("edge_img_2",edge_img_2);

    //cv::Mat edge_img_3;
    //cv::Canny(equal_img,edge_img_3,50,100);
    //cv::imshow("edge_img_3",edge_img_3);


    // find edge
    //vector<vector<cv::Point> > contours;
    //vector<cv::Vec4i> hierarchy;

    //cv::findContours( edge_img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    // for(int i = 0; i < contours.size(); i++)
    // {
    //     for(int j = 0; j < contours[i].size(); j++)
    //         std::cout << contours[i][j].x << "x" << contours[i][j].y << " ";
    //     std::cout << std::endl;
    // }


    
    // cv::waitKey(10);


    // adapt to point cloud
    //depth_img = median_filtered_img/8;













    
    std::vector<Eigen::Vector3d> pointcloud;

     for ( int m = 0 ; m <depth_img. rows ; m ++)
        for ( int n = 0 ; n <depth_img. cols ; n ++)
        {
            // Get the value at (m, n) in the depth map
            ushort d = depth_img. ptr < ushort > (m) [n];
            //std::cout << "d : " << d << std::endl;

            // Get the value at (m, n) in the edge map
            // ushort edge_1 = edge_img. ptr < ushort > (m-1) [n-1];
            // ushort edge_2 = edge_img. ptr < ushort > (m-1) [n];
            // ushort edge_3 = edge_img. ptr < ushort > (m-1) [n+1];
            // ushort edge_4 = edge_img. ptr < ushort > (m) [n-1];
            // ushort edge_5 = edge_img. ptr < ushort > (m) [n];
            // ushort edge_6 = edge_img. ptr < ushort > (m) [n+1];
            // ushort edge_7 = edge_img. ptr < ushort > (m+1) [n-1];
            // ushort edge_8 = edge_img. ptr < ushort > (m+1) [n];
            // ushort edge_9 = edge_img. ptr < ushort > (m+1) [n+1];

            // d may have no value, if so, skip this point
            // if depth is too far, it could be very noisy.
            // too solve that issue, i limit depth to 5000
            if (d == 0 || d > 5000 )
                continue ;

                

            // d exists, then add a point to the point cloud
            //if(edge_1==255 || edge_2==255 || edge_3==255 || edge_4==255 || edge_5==255 || edge_6==255 || edge_7==255 || edge_8==255 || edge_9==255)
            //    continue ;
                     
            // Calculate the spatial coordinates of this point
            float z = double (d) / 1000; // 1000 is camera factor
            float x = (n - cx) * z / fx;
            float y = (m - cy) * z / fy;

            Eigen::Vector3d point(x,y,z);

            pointcloud.push_back(point);
        }


    return pointcloud;
}

std::vector<Eigen::Vector3d> Map::pointcloud_filtering(std::vector<Eigen::Vector3d> &pointcloud)
{
    
    octomap::OcTree octree( 0.1 );
    for (auto p:pointcloud)
    {
        octree.updateNode( octomap::point3d(p[0], p[1], p[2]), true );
    }
    octree.updateInnerOccupancy();
    
    std::vector<Eigen::Vector3d> filtered_pointcloud;
    filtered_pointcloud.resize( octree.getNumLeafNodes() );
    //std::cout << "  octree.getNumLeafNodes() = " << octree.getNumLeafNodes() << std::endl;
    int i=0;
    for(octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it){
            // Fetching the coordinates in octomap-space
            // std::cout << "  x = " << it.getX() << std::endl;
            // std::cout << "  y = " << it.getY() << std::endl;
            // std::cout << "  z = " << it.getZ() << std::endl;
            // std::cout << "  size = " << it.getSize() << std::endl;
            // std::cout << "  depth = " << it.getDepth() << std::endl;
            filtered_pointcloud[i][0] = it.getX();
            filtered_pointcloud[i][1] = it.getY();
            filtered_pointcloud[i][2] = it.getZ();
            i++;
    }

    return filtered_pointcloud;
}







void Map::ICP(std::vector<std::vector<Eigen::Vector3d>> &pointcloud_data)
{
    std::cout << "pointcloud_data.size() : " << pointcloud_data.size() <<std::endl;

    // check point cloud size, at least two point cloud sould be here
    if (pointcloud_data.size() < 3) return;

    //std::cout << "pointcloud_data[0].size() : " << pointcloud_data[0].size() <<std::endl;
    //std::cout << "pointcloud_data[1].size() : " << pointcloud_data[1].size() <<std::endl;


    
    //std::cout << "eliminated_pointcloud_data[0].size() : " << pointcloud_data[0].size() <<std::endl;
    //std::cout << "eliminated_pointcloud_data[1].size() : " << pointcloud_data[1].size() <<std::endl;

    

}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> Map::computeRigidTransform(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst)
{
	assert(src.size() == dst.size());
	int pairSize = src.size();
	Eigen::Vector3d center_src(0, 0, 0), center_dst(0, 0, 0);
	for (int i=0; i<pairSize; ++i)
	{
		center_src += src[i];
		center_dst += dst[i];
	}
	center_src /= (double)pairSize;
	center_dst /= (double)pairSize;

	Eigen::MatrixXd S(pairSize, 3), D(pairSize, 3);
	for (int i=0; i<pairSize; ++i)
	{
		for (int j=0; j<3; ++j)
			S(i, j) = src[i][j] - center_src[j];
		for (int j=0; j<3; ++j)
			D(i, j) = dst[i][j] - center_dst[j];
	}
	Eigen::MatrixXd Dt = D.transpose();
	Eigen::Matrix3d H = Dt*S;
	Eigen::Matrix3d W, U, V;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd H_(3, 3);
	for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) H_(i, j) = H(i, j);
	svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV );
	if (!svd.computeU() || !svd.computeV()) {
		std::cerr << "decomposition error" << endl;
		return std::make_pair(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
	}
	Eigen::Matrix3d Vt = svd.matrixV().transpose();
	Eigen::Matrix3d R = svd.matrixU()*Vt;
	Eigen::Vector3d t = center_dst - R*center_src;	
	
	return std::make_pair(R, t);
}
