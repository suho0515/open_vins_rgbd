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


    pointcloud = depth_to_pointcloud(depth_img);


    //pointcloud_data.emplace_back(pointcloud);

    rT2 =  boost::posix_time::microsec_clock::local_time();
    
    return;
}

std::vector<Eigen::Vector3d> Map::depth_to_pointcloud(cv::Mat &depth_img) {

    float fx = camera_k_OPENCV.at(0)(0,0);
    float fy = camera_k_OPENCV.at(0)(0,2);
    float cx = camera_k_OPENCV.at(0)(1,1);
    float cy = camera_k_OPENCV.at(0)(1,2);


    //GaussianBlur(depth_img, depth_img, cv::Size(5,5),3.0);
    //medianBlur(depth_img, depth_img, 7);
    //imshow("aa",depth_img);
    //cout<<"Type : " << depth_img.type() << endl;
    //depth_img.convertTo(depth_img, CV_32F);
    //bilateralFilter(depth_img, depth_img, 10, 50, 50);
    
    std::vector<Eigen::Vector3d> pointcloud;

     for ( int m = 0 ; m <depth_img. rows ; m ++)
        for ( int n = 0 ; n <depth_img. cols ; n ++)
        {
            // Get the value at (m, n) in the depth map
            ushort d = depth_img. ptr < ushort > (m) [n];

            // d may have no value, if so, skip this point
            if (d == 0 )
                continue ;
            // d exists, then add a point to the point cloud
                     
            // Calculate the spatial coordinates of this point
            float z = double (d) / 1000; // 1000 is camera factor
            float x = (n - cx) * z / fx;
            float y = (m - cy) * z / fy;

            // 20200625, edited by suho
            // because color frame isn't same with depth frame, we need to affine pointcloud to color
            Eigen::Matrix4f affine_matrix;
            affine_matrix << 0.9999950528144836, -0.0019471179693937302, -0.0024772195611149073, 0.014739668928086758,
                      0.0019419413292780519, 0.9999959468841553, -0.0020903833210468292, 7.83317445893772e-05,
                      0.0024812796618789434, 0.002085562329739332, 0.9999947547912598, 0.0005913236527703702,
                      0.0, 0.0, 0.0, 1.0;



            Eigen::Vector4f p(x,y,z, 0.0);

            Eigen::Vector4f affined_p = affine_matrix*p;

            Eigen::Vector3d affined_p_int((int)affined_p.coeff(0),(int)affined_p.coeff(1),(int)affined_p.coeff(2));
            
            Eigen::Matrix3f rotation_matrix;
            rotation_matrix << 0.9999950528144836, -0.0019471179693937302, -0.0024772195611149073,
                      0.0019419413292780519, 0.9999959468841553, -0.0020903833210468292,
                      0.0024812796618789434, 0.002085562329739332, 0.9999947547912598;

            Eigen::Vector3f pp(x,y,z);

            Eigen::Vector3f rotated_p = rotation_matrix*pp;

            Eigen::Vector3d rotated_p_int((int)rotated_p.coeff(0),(int)rotated_p.coeff(1),(int)rotated_p.coeff(2));

            Eigen::Vector3d point(x,y,z);

            pointcloud.push_back(point);
        }


    return pointcloud;
}
