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

void Map::feed_depth(octomap::OcTree octree, double timestamp, cv::Mat &depth_img, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    //cv::imshow("depth_img",depth_img);
    // depth image filtering ( bacause there are too many noise )
    cv::Mat median_filtered_img;
    cv::medianBlur(depth_img,median_filtered_img,7);
    //cv::imshow("median_filtered_img",median_filtered_img);

    // convert depth image, for using processing
    // raw depth image is "CV_16U", it's not useful for processing
    // cv::Mat depth_img_8u;
    // double min, max;
    // cv::minMaxIdx(median_filtered_img, &min, &max);    
    // median_filtered_img.convertTo(depth_img_8u, CV_8U,255.0/(max-min),-255.0*min/(max-min));
    // cv::imshow("depth_img_8u",depth_img_8u);

    // interpolation
    // cv::Mat interpolated_img;
    // cv::resize(depth_img_8u,interpolated_img,cv::Size( depth_img_8u.cols, depth_img_8u.rows ), 0, 0, CV_INTER_LINEAR);
    // cv::imshow("interpolated_img",interpolated_img);

    // we need to care about noise of edge.
    // let's denoise it.
    // cv::Mat edge_img;
    // cv::Canny(interpolated_img,edge_img,50,100);
    // cv::imshow("edge_img",edge_img);

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


    
    // cv::Mat eroded_img;
    // cv::erode(img,eroded_img,cv::Mat());
    // cv::imshow("eroded_img",eroded_img);

    // cv::Mat dilated_img;
    // cv::dilate(img,dilated_img,cv::Mat());
    // cv::imshow("dilated_img",dilated_img);

    // cv::Mat element(7,7,CV_8U,cv::Scalar(1));
    // cv::Mat big_dilated_img;
    // cv::dilate(img,big_dilated_img,element);
    // cv::imshow("big_dilated_img",big_dilated_img);

    // cv::Mat three_dilated_img;
    // cv::dilate(big_dilated_img,three_dilated_img,cv::Mat(),cv::Point(-1,-1),3);
    // cv::imshow("three_dilated_img",three_dilated_img);

    // cv::Mat element5(3,3,CV_8U,cv::Scalar(1));
    // cv::Mat closed_img;
    // cv::morphologyEx(img,closed_img,cv::MORPH_CLOSE,element5);
    // cv::imshow("closed_img",closed_img);

    // cv::Mat opened_img;
    // cv::morphologyEx(img,opened_img,cv::MORPH_OPEN,element5);
    // cv::imshow("opened_img",opened_img);

    // filtering pointcloud
    pointcloud = pointcloud_filtering(pointcloud,0.1);
    
    // for(int n=0;n<1;n++)
    //     pointcloud = pointcloud_mean_filtering(pointcloud);

    
    //pointcloud = pointcloud_to_mat(pointcloud);
    

    pointcloud = elevation(pointcloud,"each_frame");


    



    //pointcloud_data.emplace_back(pointcloud);





    

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
            //std::cout << "y : " << y << std::endl;
            // delete ceiling and floor
            if(y < 0.0 || y > 1.0 ) continue;
            //if(y < -0.5) continue;
            if(x < -4.0 || x > 4.0 ) continue;

            Eigen::Vector3d point(x,y,z);

            pointcloud.push_back(point);
        }


    return pointcloud;
}

std::vector<Eigen::Vector3d> Map::pointcloud_filtering(std::vector<Eigen::Vector3d> &pointcloud, float octree_num)
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
            // std::cout << "  x = " << filtered_pointcloud[i][0] << std::endl;
            // std::cout << "  y = " << filtered_pointcloud[i][1] << std::endl;
            // std::cout << "  z = " << filtered_pointcloud[i][2] << std::endl;
            i++;
    }



    return filtered_pointcloud;
}


std::vector<Eigen::Vector3d> Map::pointcloud_mean_filtering(std::vector<Eigen::Vector3d> &pointcloud)
{
    std::vector<Eigen::Vector3d> filtered_pointcloud = pointcloud;
    int filter_size = 3;
    float octo_size = 0.1;
    
    std::vector<int> remove_point;
    for(int pc = 0;pc<pointcloud.size();pc++)
    {
        // std::cout << "x : " << pointcloud[pc][0] << std::endl;
        // std::cout << "y : " << pointcloud[pc][1] << std::endl;
        // std::cout << "z : " << pointcloud[pc][2] << std::endl; 
        int cnt=0;
        Eigen::Vector3d p(pointcloud[pc][0], pointcloud[pc][1], pointcloud[pc][2]);

        //std::cout << "pointcloud.size() : " << pointcloud.size() << std::endl;
        // std::cout << "p[0] : " << p[0] << std::endl;
        // std::cout << "p[1] : " << p[1] << std::endl;
        // std::cout << "p[2] : " << p[2] << std::endl;    
        for(int i=-filter_size;i<filter_size+1;i++) {
            float f_x = p[0]+(float)i*octo_size;
            for(int j=-filter_size;j<filter_size+1;j++) {
                float f_y = p[1]+(float)j*octo_size;
                for(int k=-filter_size;k<filter_size+1;k++) {
                    float f_z = p[2]+(float)k*octo_size;
                    // std::cout << "p[0] : " << p[0] << std::endl;
                    // std::cout << "p[1] : " << p[1] << std::endl;
                    // std::cout << "p[2] : " << p[2] << std::endl;    
                    // std::cout << "f_x : " << f_x << std::endl;
                    // std::cout << "f_y : " << f_y << std::endl;
                    // std::cout << "f_z : " << f_z << std::endl;
                    Eigen::Vector3d surround_point(f_x,f_y,f_z);
                    std::vector<Eigen::Vector3d>::iterator it;
                    
                    it = std::find(pointcloud.begin(),pointcloud.end(),surround_point);
                    //std::cout << "it : " << std::endl << *it << std::endl;
                    if(it != pointcloud.end()) {
                        cnt++;
                    }                    
                }
            }
        }
        double mean = (double)cnt/pow((filter_size*3),3);
        if(cnt!=0 || mean!=0.0) {
            //std::cout << "cnt : " << cnt << std::endl;
            //std::cout << "mean : " << mean << std::endl;
        }

        //if(cnt< 1) filtered_pointcloud.erase(filtered_pointcloud.begin()+pc);
        if(cnt<2) remove_point.push_back(pc);
    }
    //std::cout << "remove_point : " << remove_point << std::endl;
    for(int i = remove_point.size()-1;i>=0;i--)
    {
        filtered_pointcloud.erase(filtered_pointcloud.begin()+remove_point[i]);
    }




    return filtered_pointcloud;
}


cv::Mat Map::pointcloud_to_mat(std::vector<Eigen::Vector3d> &pointcloud, std::string name)
{
      
    // for result_pc
    float max_x = pointcloud.at(0)[0];
    float max_y = pointcloud.at(0)[1];
    float max_z = pointcloud.at(0)[2];
    float min_x = pointcloud.at(0)[0];
    float min_y = pointcloud.at(0)[1];
    float min_z = pointcloud.at(0)[2];
    for(const auto& point: pointcloud) {

        if(max_x < point[0]) max_x = point[0];
        else if(min_x > point[0]) min_x = point[0];

        if(max_y < point[1]) max_y = point[1];
        else if(min_y > point[1]) min_y = point[1];

        if(max_z < point[2]) max_z = point[2];
        else if(min_z > point[2]) min_z = point[2];
        
    }

    min_z = min_z - 0.1; // to prevent min value would be 0.0
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z; 

    int width = (int)((dx+0.1)*100.0);
    width = width - (width/10)*9;
    int height = (int)((dy+0.1)*100.0);
    height = height - (height/10)*9;
    
    float normalized_min_x = ((min_x+0.05)*10.0);
    float normalized_min_y = ((min_y+0.05)*10.0);
    float normalized_min_z = ((min_z+0.05)*10.0);

    // to prevent there are double
    normalized_min_x = round(normalized_min_x);
    normalized_min_y = round(normalized_min_y);
    normalized_min_z = round(normalized_min_z); 

    cv::Mat elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);

    for(const auto& p: pointcloud) {

        if(min_x == 0.0) min_x = 0.05;
        if(min_y == 0.0) min_y = 0.05;
        if(min_z == 0.0) min_z = 0.05;

        float x = ((p[0]+0.05)*10.0)-normalized_min_x;
        float y = ((p[1]+0.05)*10.0)-normalized_min_y;
        float z = ((p[2]+0.05)*10.0)-normalized_min_z;

        float f_z = ((p[2]-min_z)/dz)*1.0;

        // to prevent there are double
        x = round(x);
        y = round(y);
        z = round(z); 

        if(elevation_mat.at<float>((int)y,(int)x) < f_z) {
            elevation_mat.at<float>((int)y,(int)x) = f_z;
        }
    }
     
    
    
    
    // cv::imshow(name, elevation_mat);

    // cv::medianBlur(elevation_mat, elevation_mat, 3);
    // std::string median = "median "+name;
    // cv::imshow(median, elevation_mat);
    // if(name=="each_frame") {
    //     cv::erode(elevation_mat,elevation_mat,cv::Mat());
    //     std::string erode = "erode "+name;
    //     cv::imshow(erode, elevation_mat);
    // }

    //elevation_mat.release();    



    return elevation_mat;
}


std::vector<Eigen::Vector3d> Map::compare_elevation(cv::Mat result_mat, std::vector<Eigen::Vector3d> &pointcloud, std::string name, std::vector<Eigen::Vector3d> &pointcloud_each)
{
    std::vector<Eigen::Vector3d> elevated_pointcloud;

    float max_x = pointcloud.at(0)[0];
    float max_y = pointcloud.at(0)[1];
    float max_z = pointcloud.at(0)[2];
    float min_x = pointcloud.at(0)[0];
    float min_y = pointcloud.at(0)[1];
    float min_z = pointcloud.at(0)[2];

    for(const auto& point: pointcloud) {
        if(max_x < point[0]) max_x = point[0];
        else if(min_x > point[0]) min_x = point[0];

        if(max_y < point[1]) max_y = point[1];
        else if(min_y > point[1]) min_y = point[1];

        if(max_z < point[2]) max_z = point[2];
        else if(min_z > point[2]) min_z = point[2];
    }

    min_z = min_z - 0.1; // to prevent min value would be 0.0
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z; 

    int width = (int)((dx+0.1)*100.0);
    width = width - (width/10)*9;
    int height = (int)((dy+0.1)*100.0);
    height = height - (height/10)*9;
    
    float normalized_min_x = ((min_x+0.05)*10.0);
    float normalized_min_y = ((min_y+0.05)*10.0);
    float normalized_min_z = ((min_z+0.05)*10.0);

    // to prevent there are double
    normalized_min_x = round(normalized_min_x);
    normalized_min_y = round(normalized_min_y);
    normalized_min_z = round(normalized_min_z); 




    // for each_pc
    float each_max_x = pointcloud_each.at(0)[0];
    float each_max_y = pointcloud_each.at(0)[1];
    float each_max_z = pointcloud_each.at(0)[2];
    float each_min_x = pointcloud_each.at(0)[0];
    float each_min_y = pointcloud_each.at(0)[1];
    float each_min_z = pointcloud_each.at(0)[2];
    for(const auto& point: pointcloud_each) {

        if(each_max_x < point[0]) each_max_x = point[0];
        else if(each_min_x > point[0]) each_min_x = point[0];

        if(each_max_y < point[1]) each_max_y = point[1];
        else if(each_min_y > point[1]) each_min_y = point[1];

        if(each_max_z < point[2]) each_max_z = point[2];
        else if(each_min_z > point[2]) each_min_z = point[2];
        
    }

    each_min_z = each_min_z - 0.1; // to prevent min value would be 0.0
    float each_dx = each_max_x - each_min_x;
    float each_dy = each_max_y - each_min_y;
    float each_dz = each_max_z - each_min_z; 

    cv::Mat elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);

    for(const auto& p: pointcloud_each) {
        if(min_x == 0.0) min_x = 0.05;
        if(min_y == 0.0) min_y = 0.05;
        if(min_z == 0.0) min_z = 0.05;

        float x = ((p[0]+0.05)*10.0)-normalized_min_x;
        float y = ((p[1]+0.05)*10.0)-normalized_min_y;
        float z = ((p[2]+0.05)*10.0)-normalized_min_z;

        float f_z = ((p[2]-each_min_z)/each_dz)*1.0;

        // to prevent there are double
        x = round(x);
        y = round(y);
        z = round(z); 

        // std::cout << "  width = " << width << std::endl;
        // std::cout << "  height = " << height << std::endl;

        if(x > width) std::cout << "x = " << x << std::endl;
        if(y > height) std::cout << "y = " << y << std::endl;
        if(f_z > 1.0) std::cout << "f_z = " << f_z << std::endl;
        if(x < width && y < height) {
            if(elevation_mat.at<float>((int)y,(int)x) < f_z) {
                elevation_mat.at<float>((int)y,(int)x) = f_z;
            }
        }
    }
     
    cv::imshow(name, elevation_mat);

    cv::medianBlur(elevation_mat, elevation_mat, 3);
    std::string median = "median "+name;
    cv::imshow(median, elevation_mat);
    if(name=="each_frame" || name=="test") {
        cv::erode(elevation_mat,elevation_mat,cv::Mat());
        //cv::erode(elevation_mat,elevation_mat,cv::Mat());
        std::string erode = "erode "+name;
        cv::imshow(erode, elevation_mat);
    }

    cv::waitKey(10);
    
    for(int y=0; y<(int)height;y++) {
        for(int x=0; x<(int)width;x++) {
            if(result_mat.at<float>(y,x)==0.0) continue;
            
            float pc_x = ((x+normalized_min_x)/10.0)-0.05;
            float pc_y = ((y+normalized_min_y)/10.0)-0.05;
            float pc_z;

            if(elevation_mat.at<float>(y,x)==0.0 && result_mat.at<float>(y,x)!=0.0) {
                pc_z = ((result_mat.at<float>(y,x)/1.0)*dz)+min_z;
            }
            else {
                pc_z = ((elevation_mat.at<float>(y,x)/1.0)*each_dz)+each_min_z;
            }

            Eigen::Vector3d point(pc_x,pc_y,pc_z);
            elevated_pointcloud.push_back(point);
        }
    }

    elevation_mat.release();

    return elevated_pointcloud;
}

std::vector<Eigen::Vector3d> Map::elevation(std::vector<Eigen::Vector3d> &pointcloud, std::string name)
{
    // std::vector<Eigen::Vector3d> elevated_pointcloud(pointcloud.size());

    // Eigen::Vector3d zero_vec(0.0, 0.0, 0.0);
    // for(int i = 0; i<pointcloud.size();i++) {
    //     elevated_pointcloud.push_back(zero_vec);
    // }

    std::vector<Eigen::Vector3d> elevated_pointcloud;

    float max_x = pointcloud.at(0)[0];
    float max_y = pointcloud.at(0)[1];
    float max_z = pointcloud.at(0)[2];
    float min_x = pointcloud.at(0)[0];
    float min_y = pointcloud.at(0)[1];
    float min_z = pointcloud.at(0)[2];
    for(const auto& point: pointcloud) {
        // if(isinf(point[0]) || isinf(point[1]) || isinf(point[2])
        //     || fabs(point[0]) > 100.0 || fabs(point[1]) > 100.0 || fabs(point[2]) > 100.0) {
        //     std::cout << "  point[0] = " << point[0] << "  point[1] = " << point[1] << "  point[2] = " << point[2] << std::endl;
        // }
        // if(!elevate_minmax_initialize_flag) {
        //     max_x = point[0];
        //     max_y = point[1];
        //     max_z = point[2];
        //     min_x = point[0];
        //     min_y = point[1];
        //     min_z = point[2];
        //     elevate_minmax_initialize_flag = true;
        // }

        if(max_x < point[0]) max_x = point[0];
        else if(min_x > point[0]) min_x = point[0];

        if(max_y < point[1]) max_y = point[1];
        else if(min_y > point[1]) min_y = point[1];

        if(max_z < point[2]) max_z = point[2];
        else if(min_z > point[2]) min_z = point[2];
        
    }
    //elevate_minmax_initialize_flag = false;


    min_z = min_z - 0.1; // to prevent min value would be 0.0
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z; 

    // if(isinf(dx) || isinf(dy) || isinf(dz)) {
    //     elevated_pointcloud.clear();
    //     return elevated_pointcloud;
    // }

    // std::cout << "  max_x = " << max_x << std::endl;
    // std::cout << "  max_y = " << max_y << std::endl;
    // std::cout << "  max_z = " << max_z << std::endl;

    // std::cout << "  min_x = " << min_x << std::endl;
    // std::cout << "  min_y = " << min_y << std::endl;
    // std::cout << "  min_z = " << min_z << std::endl;

    // std::cout << "  dx = " << dx << std::endl;
    // std::cout << "  dy = " << dy << std::endl;
    // std::cout << "  dz = " << dz << std::endl;

    int width = (int)((dx+0.1)*100.0);
    width = width - (width/10)*9;
    int height = (int)((dy+0.1)*100.0);
    height = height - (height/10)*9;

    //std::cout << "  width = " << width << std::endl;
    //std::cout << "  height = " << height << std::endl;

    // if(width == 0 || height == 0) {
    //     elevated_pointcloud.clear();
    //     return elevated_pointcloud;
    // }

    
    float normalized_min_x = ((min_x+0.05)*10.0);
    float normalized_min_y = ((min_y+0.05)*10.0);
    float normalized_min_z = ((min_z+0.05)*10.0);

    // to prevent there are double
    normalized_min_x = round(normalized_min_x);
    normalized_min_y = round(normalized_min_y);
    normalized_min_z = round(normalized_min_z); 

    //cv::Mat elevation_mat((int)height,(int)width,CV_8U);
    //elevation_mat = cv::Scalar(254);

    cv::Mat elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);

    for(const auto& p: pointcloud) {
    //     if(isinf(min_x) || isinf(min_y) || isinf(min_z)) continue;
    //     if(p[0]==0.0 || p[1]==0.0 || p[0]==0.0) continue;
        if(min_x == 0.0) min_x = 0.05;
        if(min_y == 0.0) min_y = 0.05;
        if(min_z == 0.0) min_z = 0.05;



        // std::cout << "  p[0] = " << p[0] << std::endl;
        // std::cout << "  p[1] = " << p[1] << std::endl;
        // std::cout << "  p[2] = " << p[2] << std::endl;



        // std::cout << "  min_x = " << min_x << std::endl;
        // std::cout << "  min_y = " << min_y << std::endl;
        // std::cout << "  min_z = " << min_z << std::endl;

        // std::cout << "  min_x+0.05 = " << (min_x+0.05) << std::endl;
        // std::cout << "  min_y+0.05 = " << (min_y+0.05) << std::endl;
        // std::cout << "  min_z+0.05 = " << (min_z+0.05) << std::endl;

        // std::cout << "  normalized_min_x = " << normalized_min_x << std::endl;
        // std::cout << "  normalized_min_y = " << normalized_min_y << std::endl;
        // std::cout << "  normalized_min_z = " << normalized_min_z << std::endl;

        float x = ((p[0]+0.05)*10.0)-normalized_min_x;
        float y = ((p[1]+0.05)*10.0)-normalized_min_y;
        float z = ((p[2]+0.05)*10.0)-normalized_min_z;

        float f_z = ((p[2]-min_z)/dz)*1.0;

        //std::cout << "f_z = " << f_z << std::endl;

        // to prevent there are double
        x = round(x);
        y = round(y);
        z = round(z); 

        // std::cout << "x = " << x << std::endl;
        // std::cout << "y = " << y << std::endl;
        // std::cout << "z = " << z << std::endl;


        // std::cout << "  p[1] = " << p[1] << std::endl;
        // std::cout << "  p[1]-min_y = " << p[1]-min_y << std::endl;
        // std::cout << "  (int)((p[1]-min_y)*10.0) = " << (int)((p[1]-min_y)*10.0) << std::endl;
        // std::cout << "elevation_mat.at<int>((int)z,(int)x) :" << elevation_mat.at<int>((int)z,(int)x) << std::endl;

        if(f_z == 0.0 && p[2] != min_z )  {
            std::cout << "  p[2] = " << p[2] << std::endl;
            std::cout << "z = " << z << std::endl;
            std::cout << "  min_z = " << min_z << std::endl;
            
        }

        if(elevation_mat.at<float>((int)y,(int)x) < f_z) {
            elevation_mat.at<float>((int)y,(int)x) = f_z;

    //         //elevated_pointcloud.insert(elevated_pointcloud.begin()+((int)((width*z)+x)),p);
        }
    }
     
    
    
    
    cv::imshow(name, elevation_mat);


    if(name=="each_frame") {

        cv::medianBlur(elevation_mat, elevation_mat, 3);
        std::string median = "median "+name;
        cv::imshow(median, elevation_mat);

        cv::erode(elevation_mat,elevation_mat,cv::Mat());
        std::string erode = "erode "+name;
        cv::imshow(erode, elevation_mat);




    }
    else if(name=="filtered_frame") {

        cv::medianBlur(elevation_mat, elevation_mat, 3);
        std::string median = "median "+name;
        cv::imshow(median, elevation_mat);

        // cv::erode(elevation_mat,elevation_mat,cv::Mat());
        // std::string erode = "erode "+name;
        // cv::imshow(erode, elevation_mat);



    }



    // cv::Mat bilateral_mat;
    // cv::bilateralFilter(elevation_mat, bilateral_mat, -1,15,15);
    // std::string bilateral = "bilateral "+name;
    // cv::imshow(bilateral, bilateral_mat);
    
    
    // int nHistSize = FLT_MAX;
    // float fRange[] = { 0.0f, 1.0f + FLT_EPSILON };
    // const float* fHistRange = { fRange };
    // cv::Mat matHist;
    // cv::calcHist(&elevation_mat, 1, 0, cv::Mat(), matHist, 1, &nHistSize, &fHistRange);
    // std::string hist_name = "hist "+name;
    // cv::imshow(hist_name, matHist);


    // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    // clahe->setClipLimit(4);
    // cv::Mat elevation_mat_16u;
    // elevation_mat.convertTo(elevation_mat_16u,CV_16U);
    // cv::Mat dst;
    // clahe->apply(elevation_mat_16u, dst);
    // std::string clahe_name = "clahe "+name;
    // cv::imshow(clahe_name, dst);

    cv::waitKey(10);
    
    



    for(int y=0; y<(int)height;y++) {
        for(int x=0; x<(int)width;x++) {
            if(elevation_mat.at<float>(y,x)==0.0) continue;
                    
            float pc_x = ((x+normalized_min_x)/10.0)-0.05;
            float pc_y = ((y+normalized_min_y)/10.0)-0.05;
            float pc_z = ((elevation_mat.at<float>(y,x)/1.0)*dz)+min_z;

            // std::cout << "  pc_x = " << pc_x << std::endl;
            // std::cout << "  pc_y = " << pc_y << std::endl;
            // std::cout << "  pc_z = " << pc_z << std::endl;

            Eigen::Vector3d point(pc_x,pc_y,pc_z);
            elevated_pointcloud.push_back(point);
        }
    }

    elevation_mat.release();

    return elevated_pointcloud;
}

std::vector<Eigen::Vector3d> Map::align_pointcloud(std::vector<Eigen::Vector3d> &pointcloud)
{
    std::vector<Eigen::Vector3d> aligned_pointcloud;

    for (auto p:pointcloud)
    {

        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float res_x = 0.0;
        float res_y = 0.0;
        float res_z = 0.0;
        float div_x = 0.0;
        float div_y = 0.0;
        float div_z = 0.0;

        x = (int)(p[0]*100);
        //std::cout << "x : " << x << std::endl;
        div_x = (int)x/5;
        //std::cout << "x/5 : " << div_x << std::endl;
        res_x = (int)x%5;
        //std::cout << "x%5 : " << res_x << std::endl;
        if((int)div_x%2 == 0) {
            if((int)res_x >= 0) {
                x = x+(5-res_x);
            }
            else {
                x = x-(5+res_x);
            }
        }
        else {
            x = x - res_x;
        }
        //std::cout << "aligned_x : " << x << std::endl;
        x = x/100;

        y = (int)(p[1]*100);
        //std::cout << "y : " << y << std::endl;
        div_y = (int)y/5;
        //std::cout << "y/5 : " << div_y << std::endl;
        res_y = (int)y%5;
        //std::cout << "y%5 : " << res_y << std::endl;
        if((int)div_y%2 == 0) {
            if((int)res_y >= 0) y = y+(5-res_y);
            else y = y-(y+res_y);
        }
        else {
            y = y - res_y;
        }
        //std::cout << "aligned_y : " << y << std::endl;
        y = y/100;

        z = (int)(p[2]*100);
        //std::cout << "z : " << z << std::endl;
        div_z = (int)z/5;
        //std::cout << "z/5 : " << div_z << std::endl;
        res_z = (int)z%5;
        //std::cout << "z%5 : " << res_z << std::endl << std::endl;
        if((int)div_z%2 == 0) {
            if((int)res_z >= 0) z = z+(5-res_z);
            else z = z-(5+res_z);
        }
        else {
            z = z - res_z;
        }
        //std::cout << "aligned_z : " << z << std::endl;
        z = z/100;
        // x = (((int)(p[0]*std::pow(10.0,2)))/std::pow(10.0,2));
        // std::cout << "x : " << x << std::endl;
        // std::cout << "std::fmod(x,0.05) : " << std::fmod(x,0.05) << std::endl;

        // y = (((int)(p[1]*std::pow(10.0,2)))/std::pow(10.0,2));
        // std::cout << "y : " << y << std::endl;
        // std::cout << "std::fmod(y,0.05) : " << std::fmod(y,0.05) << std::endl;

        // z = (((int)(p[2]*std::pow(10.0,2)))/std::pow(10.0,2));
        // std::cout << "z : " << z << std::endl;
        // std::cout << "std::fmod(z,0.05) : " << std::fmod(z,0.05) << std::endl;                
        


        // res = std::fmod(p[0],0.05);
        // if(res >= 0.0) { 
        //     if(res < 0.025) x = ((int)((p[0] + (0.05-res))*std::pow(10.0,2)))/std::pow(10.0,2); 
        //     else x = ((int)((p[0] - res)*std::pow(10.0,2)))/std::pow(10.0,2); 
        // }
        // else {
        //     if(res < -0.025) x = ((int)((p[0] - (0.05+res))*std::pow(10.0,2)))/std::pow(10.0,2); 
        //     else x = ((int)((p[0] - res)*std::pow(10.0,2)))/std::pow(10.0,2); 
        // }

        // res = std::fmod(p[1],0.05);
        // if(res >= 0.0) {
        //     if(res < 0.025) y = ((int)((p[1] + (0.05-res))*std::pow(10.0,2)))/std::pow(10.0,2); 
        //     else y = ((int)((p[1] - res)*std::pow(10.0,2)))/std::pow(10.0,2);
        // }
        // else {
        //     if(res < -0.025) y = ((int)((p[1] - (0.05+res))*std::pow(10.0,2)))/std::pow(10.0,2); 
        //     else y = ((int)((p[1] - res)*std::pow(10.0,2)))/std::pow(10.0,2);
        // }

        // res = std::fmod(p[2],0.05);
        // if(res >= 0.0) {
        //     if(res < 0.025) z = ((int)((p[2] + (0.05-res))*std::pow(10.0,2)))/std::pow(10.0,2); 
        //     else z = ((int)((p[2] - res)*std::pow(10.0,2)))/std::pow(10.0,2);
        // }
        // else {
        //     if(res < -0.025) z = ((int)((p[2] - (0.05+res))*std::pow(10.0,2)))/std::pow(10.0,2); 
        //     else z = ((int)((p[2] - res)*std::pow(10.0,2)))/std::pow(10.0,2);
        // }

        //x = (((int)(p[0]*std::pow(10.0,1)))/std::pow(10.0,1))+0.05;
        //y = (((int)(p[1]*std::pow(10.0,1)))/std::pow(10.0,1))+0.05;
        //z = (((int)(p[2]*std::pow(10.0,1)))/std::pow(10.0,1))+0.05;
        Eigen::Vector3d point(x,y,z);

        aligned_pointcloud.push_back(point);

        // if(std::fmod(x,0.05)!=0.0 || std::fmod(y,0.05)!=0.0 || std::fmod(z,0.05)!=0.0)
        // {
        //     // std::cout << "p[0] : " << p[0] << std::endl;
        //     // std::cout << "p[1] : " << p[1] << std::endl;
        //     // std::cout << "p[2] : " << p[2] << std::endl; 
        //     // std::cout << "p[0]%0.05 : " << std::fmod(p[0],0.05) << std::endl;
        //     // std::cout << "p[1]%0.05 : " << std::fmod(p[1],0.05) << std::endl;
        //     // std::cout << "p[2]%0.05 : " << std::fmod(p[2],0.05) << std::endl; 
        //     // std::cout << "x : " << x << std::endl;
        //     // std::cout << "y : " << y << std::endl;
        //     // std::cout << "z : " << z << std::endl;
        //     // std::cout << "x%0.05 : " << std::fmod(x,0.05) << std::endl;
        //     // std::cout << "y%0.05 : " << std::fmod(y,0.05) << std::endl;
        //     // std::cout << "z%0.05 : " << std::fmod(z,0.05) << std::endl; 
        // }
    }

    //for(const auto& p: aligned_pointcloud) {
        //if(isinf(p[0]) || isinf(p[1]) || isinf(p[2])
        //    || fabs(p[0]) > 100.0 || fabs(p[1]) > 100.0 || fabs(p[2]) > 100.0) {
            //std::cout << "  p[0] = " << p[0] << "  p[1] = " << p[1] << "  p[2] = " << p[2] << std::endl;
        //}
    //}

    return aligned_pointcloud;
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
