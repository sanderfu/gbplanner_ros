#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

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

#include <fstream>
#include <iostream>
#include <unordered_map>

#include <eigen3/Eigen/Dense>
#include <kdtree/kdtree.h>
#include <planner_msgs/Edge.h>
#include <planner_msgs/Graph.h>
#include <planner_msgs/Vertex.h>
#include <tf/transform_datatypes.h>

#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/params.h"

class GraphManager {
 public:
  GraphManager();

  // Initialize a fresh graph.
  void reset();

  // Could change an index to break down the graph into sug-graphs.
  int generateSubgraphIndex();
  // Generate ID for new vertex.
  int generateVertexID();

  // Basic functions on graph including add new vertex and edge.
  void addVertex(Vertex* v);
  void addEdge(Vertex* v, Vertex* u, double weight);
  void removeEdge(Vertex* v, Vertex* u);

  int getNumVertices() { return graph_->getNumVertices(); }
  int getNumEdges() { return graph_->getNumEdges(); }

  Vertex* getVertex(int id) { return vertices_map_[id]; }
  void getLeafVertices(std::vector<Vertex*>& leaf_vertices);
  void findLeafVertices(const ShortestPathsReport& rep);

  bool findShortestPaths(ShortestPathsReport& rep);
  bool findShortestPaths(int source_id, ShortestPathsReport& rep);

  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<int>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<Vertex*>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order,
                       std::vector<Eigen::Vector3d>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order,
                       std::vector<StateVec>& path);
  double getShortestDistance(int target_id, const ShortestPathsReport& rep);
  int getParentIDFromShortestPath(int target_id,
                                  const ShortestPathsReport& rep);

  // Nearest neigbor lookup.
  bool getNearestVertex(const StateVec* state, Vertex** v_res);
  bool getNearestVertexInRange(const StateVec* state, double range,
                               Vertex** v_res);
  bool getNearestVertices(const StateVec* state, double range,
                          std::vector<Vertex*>* v_res);
  bool existVertexInRange(const StateVec* state, double range);

  void updateVertexTypeInRange(StateVec& state, double range);

  void convertGraphToMsg(planner_msgs::Graph& graph_msg);
  void convertMsgToGraph(const planner_msgs::Graph& graph_msg);

  void saveGraph(const std::string& path);
  void loadGraph(const std::string& path);

  // A wrapper on top of Boost Graph Lib.
  // Maintain a simple graph with IDs and weights.
  std::shared_ptr<Graph> graph_;
  // Mapping from vertex id to vertex property.
  std::unordered_map<int, Vertex*> vertices_map_;
  std::map<int, std::map<int, double>> edge_map_;  // id:  <neighbor id, edge cost>

  void getConnectedNeighbors(int vertex_id, std::vector<Vertex*>& neighbors);
  double getEdgeWeight(int u_id, int v_id);

 private:
  // Kd-tree for nearest neigbor lookup, also keep all vertices.
  kdtree* kd_tree_;
  // IDs are non-negative integer from 0 (root node)
  // and increased as adding new sub-graph or new vertices.
  int subgraph_ind_;
  int id_count_;

  // Map from local id for Boost Graph Lib to global ID including
  // <sub-graph-id,vertex-id>. This is mainly for debug purpose.
  std::unordered_map<int, std::pair<int, int>> local_id_map_;
};

#endif
