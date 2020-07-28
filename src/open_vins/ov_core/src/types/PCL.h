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
#ifndef OV_TYPE_TYPE_PCL_H
#define OV_TYPE_TYPE_PCL_H


#include "Vec.h"
#include "utils/colors.h"


namespace ov_type {


    /**
     * @brief Type that represent point cloud.
     *
     * not sure it would be included in state, for now it's just point cloud.
     */
    class PCL {

    public:

        /// Default constructor (feature is a Vec of size 3 or Vec of size 1 )
        PCL() {}

        /// Point Cloud ID (corresponds to frontend id)
        size_t _pclid;

        /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
        /// in this case we just use one depth camera.
        int _anchor_cam_id = -1;

        /// Timestamp of anchor clone
        double _anchor_clone_timestamp = -1;

        /// Boolean if this point cloud has had at least one anchor change
        bool has_had_anchor_change = false;

        /// Boolean if this point cloud should be marginalized out
        bool should_marg = false;

        /// First normalized uv coordinate bearing of this measurement (used for single depth representation)
        Eigen::Vector3d uv_norm_zero;

        /// First estimate normalized uv coordinate bearing of this measurement (used for single depth representation)
        Eigen::Vector3d uv_norm_zero_fej;

        /**
         * @brief Will return the position of the feature in the global frame of reference.
         * @param getfej Set to true to get the landmark FEJ value
         * @return Position of feature either in global or anchor frame
         */
        Eigen::Matrix<double,3,1> get_xyz(bool getfej);


        /**
         * @brief Will set the current value based on the representation.
         * @param p_FinG Position of the feature either in global or anchor frame
         * @param isfej Set to true to set the landmark FEJ value
         */
        void set_from_xyz(Eigen::Matrix<double,3,1> p_FinG, bool isfej);


    };
}



#endif //OV_TYPE_TYPE_PCL_H
