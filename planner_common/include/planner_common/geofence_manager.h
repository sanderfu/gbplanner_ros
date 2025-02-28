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
#ifndef GEOFENCE_MANAGER_H_
#define GEOFENCE_MANAGER_H_

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Polygon.h>

#include "planner_common/params.h"

namespace bg = boost::geometry;

// namespace explorer {

class Polygon2d {
  // NOTE: Vertices assigned to a polygon must follow clockwise direction
  // and a closed polygon (end vertex = start vertex)

 public:
  typedef bg::model::d2::point_xy<double> Point2dType;
  typedef bg::model::polygon<Point2dType> Polygon2dType;

  Polygon2d() {}
  Polygon2d(std::vector<Eigen::Vector2d> points);
  Polygon2d(const geometry_msgs::Polygon& poly_msg);
  Polygon2d(Polygon2dType& poly2d);

  void getCenter(Eigen::Vector2d& center);
  void getVertices(std::vector<Eigen::Vector2d>& vertices);
  bool isInside(const Eigen::Vector2d& point) const;
  bool doIntersectWithRectangle(const Eigen::Vector2d& center,
                                const Eigen::Vector2d& size) const;
  bool doIntersect(Polygon2dType poly) const;
  void convertPointToEigen(Point2dType& p, Eigen::Vector2d& e);
  void convertEigenToPoint(Eigen::Vector2d& e, Point2dType& p);

  Polygon2d& operator=(Polygon2d poly_in) {
    bg::assign(polygon_, poly_in.polygon_);
    return *this;
  }

  // Don't modify this directly to avoid incorrect format, use provided
  // constructors
  Polygon2dType polygon_;
};

struct GeofenceArea {
  enum GeofenceAreaType {
    kActive = 0,   // Active geofence area, have to check.
    kInactive = 1  // Geofence but inactive, ignore it.
  };

  int id;  // Unique ID (nonnegative number) for each geofence area
  GeofenceAreaType type;
  Polygon2d polygon;

  //
  GeofenceArea(int idt) {
    id = idt;
    type = GeofenceAreaType::kActive;
  }
  //
  GeofenceArea(int idt, Polygon2d poly) {
    id = idt;
    type = GeofenceAreaType::kActive;
    polygon = poly;
  }
  //
  GeofenceArea& operator=(GeofenceArea geo_in) {
    id = geo_in.id;
    type = geo_in.type;
    polygon = geo_in.polygon;
    return *this;
  }
};

class GeofenceManager {
 public:
  enum CoordinateStatus { kOK = 0, kViolated = 1 };
  GeofenceManager() : geofence_id_num_(0) {}
  void addGeofenceArea(const geometry_msgs::Polygon poly_msg,
                       bool merge = false);
  void addGeofenceArea(Polygon2d& poly, bool merge = false);
  void addGeofenceAreas(std::vector<Polygon2d>& poly_list, bool merge = false);
  void mergeGeofenceAreas();
  void removeGeofenceAreaWithID(int id);
  void clear();

  void getLocalGeofenceAreas(Eigen::Vector2d& current_position, double radius,
                             std::vector<GeofenceArea>& geo_list);
  void getAllGeofenceAreas(std::vector<GeofenceArea>& geo_list);

  CoordinateStatus getCoordinateStatus(const Eigen::Vector2d& position) const;
  CoordinateStatus getBoxStatus(const Eigen::Vector2d& center,
                                const Eigen::Vector2d& size) const;
  CoordinateStatus getPathStatus(const Eigen::Vector2d& start,
                                 const Eigen::Vector2d& end,
                                 const Eigen::Vector2d& box_size) const;
  int generateNewID() { return geofence_id_num_++; }
  bool loadParams(std::string ns);

 private:
  std::vector<GeofenceArea> geofence_list_;

 protected:
  int geofence_id_num_;
};

// }

#endif
