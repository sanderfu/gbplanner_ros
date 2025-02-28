#ifndef GRAPH_H_
#define GRAPH_H_

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

#include <unordered_map>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <eigen3/Eigen/Dense>
//#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>

// Result of the Djisktra shortest path calculation, one to all vertices.
struct ShortestPathsReport {
  ShortestPathsReport() : status(false), source_id(0) {}
  void reset() { status = false; }
  bool status;    // False if can not run the shortest path algorithm.
  int source_id;  // ID of the source vertex.
  // Direct parent ID related to the shortest path corresponding to each ID in
  // the id_list
  std::unordered_map<int, int> parent_id_map;
  // Shortest distance to source corresponding to each id from id_list
  std::unordered_map<int, double> distance_map;
};

class Graph {
 public:
  // Ref: .../using_adjacency_list.html#sec:choosing-graph-type
  // Example: https://theboostcpplibraries.com/boost.graph-vertices-and-edges
  //   typedef boost::adjacency_list<boost::listS, boost::listS,
  //   boost::undirectedS,
  //                                 boost::property<boost::vertex_index_t,
  //                                 int>, boost::property<boost::edge_weight_t,
  //                                 double>, boost::no_property>
  //       GraphType;
  // https://www.boost.org/doc/libs/1_37_0/libs/graph/doc/adjacency_list.html
  // Edge is set to setS type to avoid duplication in adding new edges.
  typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS,
                                boost::property<boost::vertex_index_t, int>,
                                boost::property<boost::edge_weight_t, double>,
                                boost::no_property>
      GraphType;
  // "using" as an alias; "typename" to explicitly claim that GraphType is a
  // type not class/variable...
  using VertexDescriptor = typename GraphType::vertex_descriptor;
  using EdgeDescriptor = typename GraphType::edge_descriptor;
  using EdgeDescriptorPair = typename std::pair<EdgeDescriptor, bool>;

  Graph();
  ~Graph();

  VertexDescriptor addSourceVertex(int id);
  VertexDescriptor addVertex(int id);
  EdgeDescriptorPair addEdge(int u_id, int v_id, double weight);
  void removeEdge(int u_id, int v_id);

  bool findDijkstraShortestPaths(int src_id, ShortestPathsReport& rep);

  int getVertexID(VertexDescriptor v);
  int getNumVertices();
  int getNumEdges();

  void getEdgeIterator(
      std::pair<GraphType::edge_iterator, GraphType::edge_iterator>& ei);
  std::tuple<int, int, double> getEdgeProperty(GraphType::edge_iterator e);
  void getVertexIterator(
      std::pair<GraphType::vertex_iterator, GraphType::vertex_iterator>& vi);
  int getVertexProperty(GraphType::vertex_iterator v);

  void printResults(const ShortestPathsReport& rep);
  void clear();

 private:
  GraphType graph_;
  VertexDescriptor source_;
  std::unordered_map<int, VertexDescriptor> vertex_descriptors_;
  std::vector<VertexDescriptor> vertices_;
  int num_vertices_;

  bool findDijkstraShortestPaths(VertexDescriptor& source,
                                 std::vector<VertexDescriptor>& shortest_paths,
                                 std::vector<double>& shortest_distances);
};

#endif
