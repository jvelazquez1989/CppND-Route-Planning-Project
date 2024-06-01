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
	
	// Find starting node
    // Define StartingNode as a pointer and set it to the address of the starting node
    RouteModel::Node* StartingNode = &m_Model.FindClosestNode(start_x, start_y);
    std::cout << "Starting node address = " << StartingNode << "\n"; //print ending node address for debugging
    std::cout << "Starting node x = " << (StartingNode->x) << "\n"; //print value of x
    std::cout << "Starting node y = " << (StartingNode->y) << "\n"; //print value of y

    // Find ending node
    // Define EndingNode as a pointer and set it to the address of the starting node
    RouteModel::Node* EndingNode = &m_Model.FindClosestNode(end_x, end_y);
    std::cout << "Ending node address = " << EndingNode << "\n"; //print ending node address for debugging
    std::cout << "Ending node x = " << (EndingNode->x) << "\n"; //print value of x
    std::cout << "Ending node y = " << (EndingNode->y) << "\n"; //print value of y

    //Store nodes
    RoutePlanner::start_node = StartingNode;
    RoutePlanner::end_node = EndingNode;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	// Input node is the current node
    // node->distance() finds the distance from the current node to the end node
    float distance_to_end = node->distance((*RoutePlanner::end_node));
    return distance_to_end;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    // - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
    for(int i = 0; i < current_node->neighbors.size(); i++)
    {
        //set the parent node
        current_node->neighbors[i]->parent = current_node;

        //calculate and set the h value
        // - Use CalculateHValue below to implement the h-Value calculation.
        current_node->neighbors[i]->h_value = CalculateHValue(current_node->neighbors[i]);

        //calculate and set the g value
        float distance_to_neighbor = current_node->neighbors[i]->distance(*current_node); //find the distance between the neighbor and current_node using the distance method
        current_node->neighbors[i]->g_value = current_node->g_value + distance_to_neighbor; //calculate the g value by adding the distance to neighbor to the g value of current_node

        // - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
        current_node->neighbors[i]->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
	// - Sort the open_list according to the sum of the h value and g value.
    //calculate the f value for each node in the open list
    std::vector<float> f_value = {};
    for(int i = 0; i < open_list.size(); i++)
    {f_value[i] = open_list[i]->g_value + open_list[i]->h_value;}  

    //build an index vector the size of f_value
  	std::vector<int> index_vector = {}; //define blank index vector
    for(int i = 0; i < f_value.size(); i++)
    {index_vector.push_back(i);}
  
    //begin sorting the index vector based on the value of the corresponding f_value
    for(int i = 0; i < index_vector.size(); i++)
    {
        for(int j = i + 1; j < index_vector.size(); j++)
        {
            if (f_value[index_vector[i]] < f_value[index_vector[j]])
            {
                //Swap indeces of the index vector if the current f_value is smaller than the next one in line
                int temp_val = index_vector[i];
                index_vector[i] = index_vector[j];
                index_vector[j] = temp_val;

                RouteModel::Node* temp_node = open_list[i];
                open_list[i] = open_list[j];
                open_list[j] = temp_node;
            }
        }
    }

    // - Create a pointer to the node in the list with the lowest sum.
    //RouteModel::Node *LowestFNode = open_list[index_vector[0]];
    RouteModel::Node *LowestFNode = open_list.back();

    // - Remove that node from the open_list.
    //open_list.erase(2);
    open_list.pop_back();

    // - Return the pointer.
    return LowestFNode;  

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
	bool exit = false;  
    int temp = 0;
    while(exit == false)
    {
        
        path_found.push_back(*current_node); //add the current node to the path found vector
        
        distance += current_node->distance(*current_node->parent); //add the distance between the current node to the parent node to the distance count
        
        current_node = current_node->parent; //set the next node to be added to equal the parent of the current node       

        if(current_node == start_node)
        {
            path_found.push_back(*start_node);
            exit = true;
        }           
    }

    //re-arrange the path_found vector to place the start node first
    reverse(path_found.begin(), path_found.end());

    /*
    std::cout << "start_node x = " << start_node->x << "\n";
    std::cout << "path_found (start) x = " << path_found[0].x << "\n";
    std::cout << "path_found (end) x = " << path_found[path_found.size()-1].x << "\n";
    */  
	

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

    // TODO: Implement your solution here.

}