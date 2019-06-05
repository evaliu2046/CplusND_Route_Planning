/**
 * @file route_model.cpp
 *
 * @brief:
 * 	Header file for Route Planning class.
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

#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    // RoutePlanner constructor
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {return distance;}
    // A Start Search Function
    void AStarSearch();

  private:
    // Add private variables or methods declarations here.
    RouteModel &m_Model;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;
    std::vector<RouteModel::Node *> open_list; 
    float distance;
    // Store the final Path
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node);
    // Calculate H value function
    float CalculateHValue (const RouteModel::Node * node);
    // Find the next node
    RouteModel::Node *NextNode();
    // Add Neighbors
    void AddNeighbors(RouteModel::Node * current_node);
};

#endif
