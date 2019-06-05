/**
 * @file route_model.h
 *
 * @brief:
 * 	Header file for Route Model class.
 *       
 * @ingroup:
 * 	CppND-Route-Planning-Project
 *
 * @author:
 * 	Eva Liu - evaliu2046@gmail.com
 * 
 * @date:
 * 	2019/Jun/05
 *
 */

#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H
#include <limits>
#include <cmath>
#include <unordered_map>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "model.h"

class RouteModel : public Model {
  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node * >  neighbors;
        float distance(Node other_node) const{
          return std::sqrt(std::pow((x-other_node.x),2)+ std::pow((y-other_node.y),2));
        }
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model) { node_index = idx;}
        void FindNeighbors();
      private:
        // Add private Node variables and methods here.
        int node_index;
        RouteModel * parent_model = nullptr;
        Node *FindNeighbor(std::vector<int> node_indices);
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
    auto &SNodes() { return m_Nodes; }
    auto &GetNodeToRoadMap() { return node_to_road; }
    Node &FindClosestNode(float x, float y);
  private:
    // Add private RouteModel variables and methods here.
    std::vector<Node> m_Nodes;
    std::unordered_map <int, std::vector<const Model::Road *>> node_to_road;
    void CreateNodeToRoadHashmap();
};

#endif
