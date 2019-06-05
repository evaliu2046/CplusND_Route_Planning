/**
 * @file route_model.cpp
 *
 * @brief:
 * 	C++ file for Route Model class.
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

#include "route_model.h"
#include <iostream>
#include <vector>
#include <unordered_map>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int i = 0;
  for (Model::Node node : this->Nodes()){
    m_Nodes.emplace_back(Node(i, this, node));
    i++;
  }
  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap(){
  for (const Model::Road &road: Roads()){
    if (road.type != Model::Road::Type::Footway){
      for (int node_index : Ways()[road.way].nodes){
        Model::Node node;
        if (node_to_road.find(node_index) == node_to_road.end()){
          node_to_road[node_index] = std::vector<const Model::Road*> ();
        }
        node_to_road[node_index].emplace_back(&road);
      }
    }
  }
}

RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices){
  Node *closest_node = nullptr;
  Node temp_node;
  for (int node_index : node_indices) {
    temp_node = parent_model->SNodes()[node_index];
    if ((this->distance(temp_node) != 0.0) && (!temp_node.visited)){
      if ((closest_node == nullptr) || (this->distance(temp_node)<this->distance(*closest_node))){
        closest_node = &parent_model->SNodes()[node_index];
      }  
    }
  }
  return closest_node;
}

void RouteModel::Node::FindNeighbors(){
  for (auto &road : parent_model->node_to_road[this->node_index]){
    RouteModel::Node *new_node;
    new_node = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (new_node != nullptr){
      this->neighbors.emplace_back(new_node);
    }
  }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y){
  Node temp_node;
  temp_node.x = x;
  temp_node.y = y;
  float min_dist = std::numeric_limits<float>::max();
  float dist;
  int closest_idx; //Index of the closest node
  
  for (const Model::Road &road : Roads()){
    if (road.type != Model::Road::Type::Footway){
      for (int node_index : Ways()[road.way].nodes){
        dist = temp_node.distance(SNodes()[node_index]);
        if (dist < min_dist){
          closest_idx = node_index;
          min_dist = dist;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}
