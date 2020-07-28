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
#ifndef OV_MAP_H
#define OV_MAP_H

#include <boost/filesystem.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <unordered_map>
#include <Eigen/StdVector>

// 20200625, edited by suho
// for affin matrix 
#include <Eigen/Geometry>

// 20200721, edited by suho
// for icp process
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "utils/quat_ops.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeIterator.hxx>

#include "track/TrackKLT.h"

#include "core/VioManagerOptions.h"

namespace ov_msckf {


    /**
     * @brief Mapping using rgbd camera
     *
     * Mapping class contain with functions below.
     * 1. store every point cloud 
     * 2. convert depth image to point cloud
     * 3. locate point cloud with state from ov_msckf
     */
    class Map {

    public:

        /**
         * @brief Public default constructor
         */
        Map();

        /**
         * @brief Given a the camera intrinsic values, this will set what we should normalize points with.
         * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
         */
        void set_calibration(std::map<size_t,Eigen::VectorXd> camera_calib);

        /**
         * @brief Process a new depth image
         * @param timestamp timestamp the new image occurred at
         * @param depth_img new cv:Mat depth image
         * @param cam_id the camera id that this new image corresponds too
         */
        void feed_depth(double timestamp, cv::Mat &depth_img, size_t cam_id);

        /**
         * @brief Return pointcloud 
         */
        std::vector<Eigen::Vector3d> get_pointcloud() {
            return pointcloud;
        }

        void set_pointcloud(std::vector<Eigen::Vector3d> modified_pointcloud) {
            pointcloud = modified_pointcloud;
        }

        /**
         * @brief Return stored pointcloud data 
         */
        std::vector<std::vector<Eigen::Vector3d>> get_pointcloud_data() {
            return pointcloud_data;
        }

        /**
         * @brief Return computed rigid transform of two point cloud
         */
        std::pair<Eigen::Matrix3d, Eigen::Vector3d> get_rigid_transform(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst) {
            return computeRigidTransform(src,dst);
        }

        /**
         * @brief point cloud filtering
         * @param pointcloud raw point cloud data
         */
        std::vector<Eigen::Vector3d> pointcloud_filtering(std::vector<Eigen::Vector3d> &pointcloud);


    protected:

        /**
         * @brief feature detection for depth image
         * @param depth_img new cv:Mat CV_16U image
         */
        void feature_detection(std::vector<cv::Mat> &img0pyr, std::vector<cv::KeyPoint> &pts0);



        /**
         * @brief convert depth image from rgbd to pointcloud
         * @param depth_img new cv:Mat CV_16U image
         */
        std::vector<Eigen::Vector3d> depth_to_pointcloud(cv::Mat &depth_img);




        

        /**
         * @brief doing icp for accurate pose estimate and map restruction
         * @param pointcloud_data all the pointcloud data
         */
        void ICP(std::vector<std::vector<Eigen::Vector3d>> &pointcloud_data);

        /**
         * @brief describe it later, reffered from https://gist.github.com/JiaxiangZheng/8168862 
         */
        std::pair<Eigen::Matrix3d, Eigen::Vector3d> computeRigidTransform(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst);

        // Timing statistic file and variables  
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7, rT8, rT9;

        // Camera intrinsics in OpenCV format
        std::map<size_t, cv::Matx33d> camera_k_OPENCV;

        // Camera distortion in OpenCV format
        std::map<size_t, cv::Vec4d> camera_d_OPENCV;

        // Depth camera intrinsics in OpenCV format
        std::map<size_t, cv::Matx33d> depth_camera_k_OPENCV;

        std::vector<Eigen::Vector3d> pointcloud;

        // Our history of pointcloud from depth image
        std::vector<std::vector<Eigen::Vector3d>> pointcloud_data; 

        /// Our sparse feature tracker (klt or descriptor)
        TrackBase* trackFEATS = nullptr;

        std::vector<cv::KeyPoint> last_pts;
        cv::Mat last_depth_img;
        std::vector<cv::Mat> last_imgpyr;

    };


}


#endif /* OV_MAP_H */
