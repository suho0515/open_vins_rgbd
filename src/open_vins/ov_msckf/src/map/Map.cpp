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

    // depth image filtering ( bacause there are too many noise )
    cv::Mat median_filtered_img;
    cv::medianBlur(depth_img,median_filtered_img,7);
    //cv::imshow("median_filtered_img",median_filtered_img);

    //cv::waitKey(10);

    // getting point cloud data from depth image
    pointcloud = depth_to_pointcloud(median_filtered_img);

    // filtering pointcloud (octomap)
    pointcloud = pointcloud_filtering(pointcloud,0.1);
    
    // for(int n=0;n<1;n++)
    //     pointcloud = pointcloud_mean_filtering(pointcloud);
    
    //pointcloud = pointcloud_to_mat(pointcloud);
    
    // elevation pointcloud
    //pointcloud = elevation(pointcloud,"each_frame");
    //std::cout << "pointcloud.size() : " << pointcloud.size() << std::endl; 


    



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

    std::vector<Eigen::Vector3d> pointcloud;

     for ( int m = 0 ; m <depth_img. rows ; m ++)
        for ( int n = 0 ; n <depth_img. cols ; n ++)
        {
            // Get the value at (m, n) in the depth map
            ushort d = depth_img. ptr < ushort > (m) [n];
            //std::cout << "d : " << d << std::endl;

            // d may have no value, if so, skip this point
            // if depth is too far, it could be very noisy.
            // too solve that issue, i limit depth to 5000
            if (d == 0) continue;
            if (d > 2000) continue;
                     
            // Calculate the spatial coordinates of this point
            float z = double (d) / 1000; // 1000 is camera factor
            float x = (n - cx) * z / fx;
            float y = (m - cy) * z / fy;
            //std::cout << "y : " << y << std::endl;
            // delete ceiling and floor
            //if(y < 0.0 || y > 1.0 ) continue;
            if(x < -2.0 || x > 2.0 ) continue;

            Eigen::Vector3d point(x,y,z);

            pointcloud.push_back(point);
        }
    return pointcloud;
}

std::vector<Eigen::Vector3d> Map::pointcloud_filtering(std::vector<Eigen::Vector3d> &pointcloud, float octree_num)
{
    
    octomap::OcTree octree( 0.05 );
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
        //cv::erode(elevation_mat,elevation_mat,cv::Mat());
        //std::string erode = "erode "+name;
        //cv::imshow(erode, elevation_mat);
        cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
        cv::Mat opened;
        cv::morphologyEx(elevation_mat,elevation_mat,cv::MORPH_OPEN,element5);
        std::string open = "open "+name;
        cv::imshow(open, elevation_mat);
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


    min_z = min_z - 0.05; // to prevent min value would be 0.0
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

    // calculate width anf height of image.
    // to prevent edge issue, we add to residual number to dx
    // if resolution is 0.1, add 0.1 to dx. and multiply 100.0 to dx to make it intager.
    // if resolution is 0.05, add 0.05 to dx. and multiply 1000.0 to dx to make it integer.
    // int width = (int)((dx+0.05)*1000.0);
    // width = width - (width/10)*9;
    // int height = (int)((dy+0.05)*1000.0);
    // height = height - (height/10)*9;

    int width = (int)(((dx+0.1)*100.0)/5.0);
    int height = (int)(((dy+0.1)*100.0)/5.0);

    //std::cout << "  width = " << width << std::endl;
    //std::cout << "  height = " << height << std::endl;

    // if(width == 0 || height == 0) {
    //     elevated_pointcloud.clear();
    //     return elevated_pointcloud;
    // }

    // if resolution is 0.05, point would be end with 0.025.
    // 0.025+0.025 is 0.05, so if we want to make the 3d point value to image value,
    // add 0.025 to min_x (=0.05), then multiply 100(=5), then divide with 5.0(=1).
    float normalized_min_x = ((min_x+0.025)*100.0)/5.0;
    float normalized_min_y = ((min_y+0.025)*100.0)/5.0;
    float normalized_min_z = ((min_z+0.025)*100.0)/5.0;

    // to prevent there are double
    normalized_min_x = round(normalized_min_x);
    normalized_min_y = round(normalized_min_y);
    normalized_min_z = round(normalized_min_z); 

    //cv::Mat elevation_mat((int)height,(int)width,CV_8U);
    //elevation_mat = cv::Scalar(254);

    cv::Mat tmp_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);




    for(const auto& p: pointcloud) {
        if(name=="localization_frame") {
            if(p[2] <= 0.275 || p[2] >= 0.375) continue;
        }

        // robot lidar height is 300mm, and we don't need higher than lidar height.
        // that's why we limit p[2] data.
        if(p[2] > 0.325) continue;

    //     if(isinf(min_x) || isinf(min_y) || isinf(min_z)) continue;
    //     if(p[0]==0.0 || p[1]==0.0 || p[0]==0.0) continue;
        if(min_x == 0.0) min_x = 0.025;
        if(min_y == 0.0) min_y = 0.025;
        if(min_z == 0.0) min_z = 0.025;



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

        float x = ((p[0]+0.025)*100.0)/5.0-normalized_min_x;
        float y = ((p[1]+0.025)*100.0)/5.0-normalized_min_y;
        float z = ((p[2]+0.025)*100.0)/5.0-normalized_min_z;

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

        // if(f_z == 0.0 && p[2] != min_z )  {
        //     std::cout << "  p[2] = " << p[2] << std::endl;
        //     std::cout << "z = " << z << std::endl;
        //     std::cout << "  min_z = " << min_z << std::endl;
            
        // }

        if(tmp_mat.at<float>((int)y,(int)x) < f_z) {
            tmp_mat.at<float>((int)y,(int)x) = f_z;
            
        }
    }


    
    //cv::imshow(name, elevation_mat);
    //cv::imwrite( "../01_elevation_image.bmp", elevation_mat);
    //cv::imshow("binaty_mat", binary_mat);
    


    if(name=="each_frame") {
        elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        erode_binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        erode_elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        median_elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);       

        elevation_mat = tmp_mat;

        for(int y=0; y<(int)height; y++) {
            for(int x=0; x<(int)width; x++) {
                if(elevation_mat.at<float>((int)y,(int)x) != 0.0) {
                    binary_mat.at<uchar>((int)y,(int)x) = 255;
                }
            }
        }
        //cv::imwrite( "../02_binary_image.bmp", binary_mat);
        //cv::imshow("binary_mat", binary_mat);

        // cv::erode(elevation_mat,elevation_mat,cv::Mat());
        // std::string erode = "erode "+name;
        // cv::imshow(erode, elevation_mat);

        //cv::erode(binary_mat,binary_mat,cv::Mat());
        cv::erode(binary_mat,erode_binary_mat,cv::Mat());
        //std::string erode_binary = "erode binary "+name;
        //cv::imshow(erode_binary, erode_binary_mat);
        //cv::imwrite( "../03_erode_binary_image.bmp", binary_mat);

        erode_elevation_mat = elevation_mat.clone();
        for(int y=0; y<(int)height; y++) {
            for(int x=0; x<(int)width; x++) {
                if(erode_binary_mat.at<uchar>((int)y,(int)x) == 0) {
                    erode_elevation_mat.at<float>((int)y,(int)x) = 0.0;
                }
            }
        }
        //cv::imwrite( "../04_erode_elevation_image.bmp", elevation_mat);
        //std::string erode = "erode "+name;
        //cv::imshow(erode, erode_elevation_mat);



        cv::medianBlur(erode_elevation_mat, median_elevation_mat, 3);
        //std::string median = "median "+name;
        //cv::imshow(median, elevation_mat);
        //cv::imwrite( "../05_median_elevation_image.bmp", elevation_mat);




        // cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
        // cv::Mat opened;
        // cv::morphologyEx(elevation_mat,elevation_mat,cv::MORPH_OPEN,element5);
        // std::string open = "open "+name;
        // cv::imshow(open, elevation_mat);


        tmp_mat = median_elevation_mat;
    }
    else if(name=="localization_frame") {
        elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        erode_binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        erode_elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);

        elevation_mat = tmp_mat;

        for(int y=0; y<(int)height; y++) {
            for(int x=0; x<(int)width; x++) {
                if(elevation_mat.at<float>((int)y,(int)x) != 0.0) {
                    binary_mat.at<uchar>((int)y,(int)x) = 255;
                }
            }
        }

        cv::erode(binary_mat,erode_binary_mat,cv::Mat());

        erode_elevation_mat = elevation_mat.clone();
        for(int y=0; y<(int)height; y++) {
            for(int x=0; x<(int)width; x++) {
                if(erode_binary_mat.at<uchar>((int)y,(int)x) == 0) {
                    erode_elevation_mat.at<float>((int)y,(int)x) = 0.0;
                }
            }
        }

        tmp_mat = erode_elevation_mat;
    }
    else if(name=="filtered_frame") {

        // for(int y=0; y<(int)height; y++) {
        //     for(int x=0; x<(int)width; x++) {
        //         if(elevation_mat.at<float>((int)y,(int)x) != 0.0) {
        //             binary_mat.at<uchar>((int)y,(int)x) = 255;
        //         }
        //     }
        // }

        // cv::erode(binary_mat,binary_mat,cv::Mat());
        // std::string erode = "erode "+name;
        // cv::imshow(erode, binary_mat);

        // for(int y=0; y<(int)height; y++) {
        //     for(int x=0; x<(int)width; x++) {
        //         if(binary_mat.at<uchar>((int)y,(int)x) == 0) {
        //             elevation_mat.at<float>((int)y,(int)x) = 0.0;
        //         }
        //     }
        // }




        //cv::medianBlur(elevation_mat, elevation_mat, 3);
        // std::string median = "median "+name;
        // cv::imshow(median, elevation_mat);

        // cv::erode(elevation_mat,elevation_mat,cv::Mat());
        // std::string erode = "erode "+name;
        // cv::imshow(erode, elevation_mat);

        //cv::imshow("tmp_mat", tmp_mat);
        //cv::waitKey(10);



    }
    else if(name=="result_map") {
        //std::string result = "result "+name;
        //cv::imshow(result, elevation_mat);
        //cv::imwrite( "../06_result_image.bmp", elevation_mat);
        result_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        result_mat = tmp_mat;
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
            if(tmp_mat.at<float>(y,x)==0.0) continue;
                    
            float pc_x = (((x+normalized_min_x)*5.0)/100.0)-0.025;
            float pc_y = (((y+normalized_min_y)*5.0)/100.0)-0.025;
            float pc_z = ((tmp_mat.at<float>(y,x)/1.0)*dz)+min_z;

            // std::cout << "  pc_x = " << pc_x << std::endl;
            // std::cout << "  pc_y = " << pc_y << std::endl;
            // std::cout << "  pc_z = " << pc_z << std::endl;
            //if(pc_z > 1.0) continue;
            //if(pc_z < -0.5) continue;
            //if(pc_z < -0.1) pc_z = -0.1;
            Eigen::Vector3d point(pc_x,pc_y,pc_z);
            elevated_pointcloud.push_back(point);
        }
    }

    tmp_mat.release();

    return elevated_pointcloud;
}

std::vector<Eigen::Vector3d> Map::localization_elevation(std::vector<Eigen::Vector3d> &pointcloud,std::vector<Eigen::Vector3d> &pointcloud_ref, std::string name)
{
    std::vector<Eigen::Vector3d> elevated_pointcloud;

    float max_x = pointcloud_ref.at(0)[0];
    float max_y = pointcloud_ref.at(0)[1];
    float max_z = pointcloud.at(0)[2];
    float min_x = pointcloud_ref.at(0)[0];
    float min_y = pointcloud_ref.at(0)[1];
    float min_z = pointcloud.at(0)[2];
    for(const auto& point: pointcloud_ref) {
        if(max_x < point[0]) max_x = point[0];
        else if(min_x > point[0]) min_x = point[0];

        if(max_y < point[1]) max_y = point[1];
        else if(min_y > point[1]) min_y = point[1];
    }

    for(const auto& point: pointcloud) {
        if(max_z < point[2]) max_z = point[2];
        else if(min_z > point[2]) min_z = point[2];
    }

    min_z = min_z - 0.05; // to prevent min value would be 0.0
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z; 

    // calculate width anf height of image.
    // to prevent edge issue, we add to residual number to dx
    // if resolution is 0.1, add 0.1 to dx. and multiply 100.0 to dx to make it intager.
    // if resolution is 0.05, add 0.05 to dx. and multiply 1000.0 to dx to make it integer.
    int width = (int)((dx+0.05)*1000.0);
    width = width - (width/10)*9;
    int height = (int)((dy+0.05)*1000.0);
    height = height - (height/10)*9;

    //std::cout << "  width = " << width << std::endl;
    //std::cout << "  height = " << height << std::endl;

    // if(width == 0 || height == 0) {
    //     elevated_pointcloud.clear();
    //     return elevated_pointcloud;
    // }

    // if resolution is 0.05, point would be end with 0.025.
    // 0.025+0.025 is 0.05, so if we want to make the 3d point value to image value,
    // add 0.025 to min_x (=0.05), then multiply 100(=5), then divide with 5.0(=1).
    float normalized_min_x = ((min_x+0.025)*100.0)/5.0;
    float normalized_min_y = ((min_y+0.025)*100.0)/5.0;
    float normalized_min_z = ((min_z+0.025)*100.0)/5.0;

    // to prevent there are double
    normalized_min_x = round(normalized_min_x);
    normalized_min_y = round(normalized_min_y);
    normalized_min_z = round(normalized_min_z); 

    cv::Mat tmp_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);




    for(const auto& p: pointcloud) {
        if(name=="localization_frame") {
            if(p[2] <= 0.275 || p[2] >= 0.375) continue;
        }

        // robot lidar height is 300mm, and we don't need higher than lidar height.
        // that's why we limit p[2] data.
        if(p[2] > 0.325) continue;

        if(min_x == 0.0) min_x = 0.025;
        if(min_y == 0.0) min_y = 0.025;
        if(min_z == 0.0) min_z = 0.025;

        float x = ((p[0]+0.025)*100.0)/5.0-normalized_min_x;
        float y = ((p[1]+0.025)*100.0)/5.0-normalized_min_y;
        float z = ((p[2]+0.025)*100.0)/5.0-normalized_min_z;

        float f_z = ((p[2]-min_z)/dz)*1.0;

        // to prevent there are double
        x = round(x);
        y = round(y);
        z = round(z); 

        if(tmp_mat.at<float>((int)y,(int)x) < f_z) {
            tmp_mat.at<float>((int)y,(int)x) = f_z;
            
        }
    }

    //cv::imshow(name, elevation_mat);
    //cv::imwrite( "../01_elevation_image.bmp", elevation_mat);
    //cv::imshow("binaty_mat", binary_mat);
    
    if(name=="each_frame") {
        elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        erode_binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        erode_elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        median_elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);       

        elevation_mat = tmp_mat;

        for(int y=0; y<(int)height; y++) {
            for(int x=0; x<(int)width; x++) {
                if(elevation_mat.at<float>((int)y,(int)x) != 0.0) {
                    binary_mat.at<uchar>((int)y,(int)x) = 255;
                }
            }
        }
        //cv::imwrite( "../02_binary_image.bmp", binary_mat);
        //cv::imshow("binary_mat", binary_mat);

        // cv::erode(elevation_mat,elevation_mat,cv::Mat());
        // std::string erode = "erode "+name;
        // cv::imshow(erode, elevation_mat);

        //cv::erode(binary_mat,binary_mat,cv::Mat());
        cv::erode(binary_mat,erode_binary_mat,cv::Mat());
        //std::string erode_binary = "erode binary "+name;
        //cv::imshow(erode_binary, erode_binary_mat);
        //cv::imwrite( "../03_erode_binary_image.bmp", binary_mat);

        erode_elevation_mat = elevation_mat.clone();
        for(int y=0; y<(int)height; y++) {
            for(int x=0; x<(int)width; x++) {
                if(erode_binary_mat.at<uchar>((int)y,(int)x) == 0) {
                    erode_elevation_mat.at<float>((int)y,(int)x) = 0.0;
                }
            }
        }
        //cv::imwrite( "../04_erode_elevation_image.bmp", elevation_mat);
        //std::string erode = "erode "+name;
        //cv::imshow(erode, erode_elevation_mat);

        //cv::medianBlur(erode_elevation_mat, median_elevation_mat, 3);
        //std::string median = "median "+name;
        //cv::imshow(median, elevation_mat);
        //cv::imwrite( "../05_median_elevation_image.bmp", elevation_mat);

        tmp_mat = erode_elevation_mat;
    }
    else if(name=="localization_frame") {
        // elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        // binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        // erode_binary_mat = cv::Mat::zeros((int)height,(int)width,CV_8U);
        // erode_elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);

        // elevation_mat = tmp_mat;

        // for(int y=0; y<(int)height; y++) {
        //     for(int x=0; x<(int)width; x++) {
        //         if(elevation_mat.at<float>((int)y,(int)x) != 0.0) {
        //             binary_mat.at<uchar>((int)y,(int)x) = 255;
        //         }
        //     }
        // }

        // cv::erode(binary_mat,erode_binary_mat,cv::Mat());

        // erode_elevation_mat = elevation_mat.clone();
        // for(int y=0; y<(int)height; y++) {
        //     for(int x=0; x<(int)width; x++) {
        //         if(erode_binary_mat.at<uchar>((int)y,(int)x) == 0) {
        //             erode_elevation_mat.at<float>((int)y,(int)x) = 0.0;
        //         }
        //     }
        // }

        // tmp_mat = erode_elevation_mat;
    }
    else if(name=="result_map") {
        //std::string result = "result "+name;
        //cv::imshow(result, elevation_mat);
        //cv::imwrite( "../06_result_image.bmp", elevation_mat);
        result_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);
        result_mat = tmp_mat;
    }

    for(int y=0; y<(int)height;y++) {
        for(int x=0; x<(int)width;x++) {
            if(tmp_mat.at<float>(y,x)==0.0) continue;
                    
            float pc_x = (((x+normalized_min_x)*5.0)/100.0)-0.025;
            float pc_y = (((y+normalized_min_y)*5.0)/100.0)-0.025;
            float pc_z = ((tmp_mat.at<float>(y,x)/1.0)*dz)+min_z;

            Eigen::Vector3d point(pc_x,pc_y,pc_z);
            elevated_pointcloud.push_back(point);
        }
    }

    tmp_mat.release();

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

void Map::clear_memory () {
    elevation_mat.release();
    binary_mat.release();
    erode_binary_mat.release();
    erode_elevation_mat.release();
    median_elevation_mat.release();
    result_mat.release();
    //tmp_mat.release();
}
