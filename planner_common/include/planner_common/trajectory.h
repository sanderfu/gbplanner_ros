#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

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

#include <nav_msgs/Path.h>

#include "planner_common/params.h"

// Class to collect all functions to manipulate trajectory.
class Trajectory {
 public:
  // For now test with simple Pose msg.
  // But later should use stamped msg with header frame instead.
  typedef geometry_msgs::Pose WayPointType;
  typedef std::vector<WayPointType> TrajectoryType;
  typedef geometry_msgs::PoseStamped WayPointStampedType;
  typedef std::vector<WayPointStampedType> TrajectoryStampedType;
  // Consider other types without orientation.
  typedef Eigen::Vector3d VectorType;
  typedef std::vector<VectorType> PathType;

  // Assume two paths with be executed with same velocity.
  // Return true if they are similar within the threshold; false otherwise.
  static bool compareTwoTrajectories(const TrajectoryType& traj_1,
                                     const TrajectoryType& traj_2,
                                     double dist_threshold,
                                     double discrete_length = 0.2,
                                     bool shorten_to_same_length = true,
                                     bool scale_with_length = true);

  static bool compareTwoTrajectories(const PathType& path_1,
                                     const PathType& path_2,
                                     double dist_threshold = 1.0,
                                     double discrete_length = 0.2,
                                     bool shorten_to_same_length = true,
                                     bool scale_with_length = true);
  // In this case we prefer not scale with path length to amplify difference
  // between long paths.
  static double computeDistanceBetweenTrajectoryAndDirection(
      const PathType& path, double heading, double discrete_length = 0.2,
      bool scale_with_length = false);

  //
  static double computeDistanceBetweenTwoTrajectories(
      const PathType& path_1, const PathType& path_2,
      double discrete_length = 0.2, bool shorten_to_same_length = true,
      bool scale_with_length = true);

  // Distance between 2 paths using Dynamic Time Warping.
  static double computeDTWDistance(const PathType& pa, const PathType& pb);

  // Interpolate trajectory based on length only.
  static bool interpolatePath(const PathType& path, double discrete_length,
                              PathType& path_intp);
  static bool interpolatePath(const TrajectoryType& traj,
                              double discrete_length,
                              TrajectoryType& traj_intp);

  //
  static void extractPathFromTrajectory(const TrajectoryType& traj,
                                        PathType& path);

  //
  static void shortenPath(PathType& path, double max_len);

  //
  static double getPathLength(const PathType& path);
  static double getPathLength(const TrajectoryType& traj);

  // Approximate heading direction of the 3D path.
  static double estimateDirectionFromPath(const PathType& path);

  //
  static void TestDTW();

 private:
  static void truncateYaw(double& x) {
    if (x > M_PI)
      x -= 2 * M_PI;
    else if (x < -M_PI)
      x += 2 * M_PI;
  }
};

// }  // namespace explorer

#endif
