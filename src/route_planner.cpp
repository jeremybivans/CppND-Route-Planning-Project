#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
	RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
  	RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	
  	//returns float value since each value of the vector is a float
  	float h_value = node->distance(*end_node);
  return	h_value;
  
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

  //Run FindNeighbors function on current_node to get all neighbors
  current_node->FindNeighbors();
  
  for (auto neighbor: current_node->neighbors){
  
    //Parent is the node before the neighbor node
  	neighbor->parent = current_node;
    //H value comes from a function
    neighbor->h_value = CalculateHValue(neighbor);
    //G value is whatever the current nodes G value + the distance between the two since its traveling over distance and not just an arbitrary 1 square
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
  
    //Checking a neighbors values means we 'visited it'
    neighbor->visited = true;
    //More efficient than just using push_back
    open_list.emplace_back(neighbor);
    
  }
  
    
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b){
	
  	//adapted Compare function from exercises to compare g and h values of nodes, returns the smaller f value
	float f1 = a->g_value + a->h_value;
  	float f2 = b->g_value + b->h_value;
  return f1 < f2;

}

RouteModel::Node *RoutePlanner::NextNode() {
  
  	//sorts the list using the Compare method made above by F-value
	sort(open_list.begin(),open_list.end(),Compare);
 //puts the best node to the beginning of the open_list, received mentor assistance on this part
  RouteModel::Node *nNode = open_list.front();
  //erases the best node after setting n(ext)Node to the front of the open_list
  open_list.erase(open_list.begin());
  	
  return nNode;

}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
	//Goes until we reach the starting node
  	while(current_node->parent != nullptr){
    	
      //
    	path_found.emplace_back(*current_node);
      //Sets the parent to current node since we are moving backwards
      auto parent = *(current_node->parent);
      //Distance will be equal to a running sum of the current node back to its parent
      distance += current_node ->distance(parent);
      //sets the current node to the parent to recursively get to the beginning of the path
      current_node = current_node->parent;
    
    }
  
 	path_found.push_back(*current_node);
	reverse(path_found.begin(),path_found.end());
  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

  	current_node = start_node;
  
  	current_node->visited = true;
	//open_list.emplace_back(current_node);
 	//AddNeighbors(current_node);
  	open_list.insert(open_list.begin(), current_node);
    // TODO: Implement your solution here.
  	while (open_list.size() > 0){
    
      	
    	current_node = NextNode();
      
      	if (current_node->distance(*end_node) == 0){
        
        	m_Model.path = ConstructFinalPath(current_node);
        	
          return;
        }
   
      	AddNeighbors(current_node);
      
    }

}
