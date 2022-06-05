#ifndef MAP_MANAGER_H_
#define MAP_MANAGER_H_

/*
BSD 3-Clause License

Copyright (c) 2020, UNR Autonomous Robots Lab
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <eigen3/Eigen/Dense>

#include "planner_common/params.h"

// namespace explorer {

class MapManager {
 public:
  enum VoxelStatus { kUnknown = 0, kOccupied, kFree };

  MapManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private) {}
  virtual double getResolution() const = 0;
  virtual bool getStatus() const = 0;
  virtual VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const = 0;
  virtual VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel) const = 0;
  virtual VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel,
                                   Eigen::Vector3d& end_voxel,
                                   double& tsdf_dist) const = 0;
  virtual VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                                   const Eigen::Vector3d& size,
                                   bool stop_at_unknown_voxel) const = 0;
  virtual VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                                    const Eigen::Vector3d& end,
                                    const Eigen::Vector3d& box_size,
                                    bool stop_at_unknown_voxel) const = 0;
  virtual bool augmentFreeBox(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& box_size) = 0;

  virtual void getScanStatus(
      Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
      std::tuple<int, int, int>& gain_log,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      SensorParamsBase& sensor_params) = 0;

  virtual void augmentFreeFrustum() = 0;

  virtual void extractLocalMap(const Eigen::Vector3d& center,
                               const Eigen::Vector3d& bounding_box_size,
                               std::vector<Eigen::Vector3d>& occupied_voxels,
                               std::vector<Eigen::Vector3d>& free_voxels) = 0;

  virtual void extractLocalMapAlongAxis(
      const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& bounding_box_size,
      std::vector<Eigen::Vector3d>& occupied_voxels,
      std::vector<Eigen::Vector3d>& free_voxels) = 0;

  virtual void resetMap() = 0;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};

// }  // namespace explorer

#endif
