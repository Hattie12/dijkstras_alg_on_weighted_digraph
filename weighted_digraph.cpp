#include "weighted_digraph.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <vector>
#include <map>
#include <iterator>
#include <queue>
#include <set>
#include <algorithm>
#include <numeric>
#include <limits>

using namespace std;

weighted_digraph::weighted_digraph(const string& filename) : numVertices(0), numArcs(0) {
  //code removed bc I didn't write this part

	std::vector<map<int, double>> init_digraph(numVertices);
	this->digraph = init_digraph;

	//code removed bc I didn't write this part
}

//inputs: node to go from, node to go to, weight of arc
//outputs: none
//side effects: adds a map of the to-node and weight at the index of the vector<map> that belongs
//to the from-node
void weighted_digraph::insert_arc(int from, int to, double weight) {

	//add the new path to the map
	(this->digraph)[from][to] = weight;
	this->numArcs += 1;
	
}
 
//inputs: path
//outputs: weight of path, or inf if path is broken
double weighted_digraph::get_path_weight(const list<int> & path) const {
	double weight = 0;

	//itrs point to start node and end node for each edge
	std::list<int>::const_iterator from_itr = path.cbegin();
	std::list<int>::const_iterator to_itr = path.cbegin();
	to_itr++;

	bool is_broken = false;

	//find weight for every edge in the path.
	for (; to_itr != path.cend(); to_itr++, from_itr++) {

		//check whether the path exists
		if (this->digraph[*from_itr].find(*to_itr) == this->digraph[*from_itr].cend()) {
			is_broken = true;
			break;
		}

		//save this edge weight
		weight += this->digraph[*from_itr].find(*to_itr)->second;
	}

	//decide whether to return inf or the weight
	auto weight_or_inf = [&weight](bool is_broken)->double
							{
							if (is_broken) {
								return std::numeric_limits<double>::infinity();
							}
							else { return weight; }
							};

	return weight_or_inf(is_broken);
}

//inputs: node to go from, node to go to
//outputs: whether there is a path between the nodes
//details: determined by whether the to-node is in the binary search tree from the from-node
bool weighted_digraph::does_path_exist(int from, int to) const {
	//search queue with the first node added
	std::queue<int> node_q;
	node_q.push(from);

	//recorded visited nodes
	std::vector<int> visited_nodes;

	bool path_exists_flag = false;

	//for checking whether the node was visited (bc I got a weird error using std::find)
	bool node_was_visited;

	//if there is nodes in the queue
	while (!node_q.empty()) {
		//extract the next node to check
		int node_to_check = node_q.front();
		node_q.pop();

		//see whether the node has already been visited
		node_was_visited = false;
		for_each(begin(visited_nodes), end(visited_nodes),
			[&node_was_visited, &node_to_check](int node) {
				if (node == node_to_check) { 
					node_was_visited = true; }; });

		//if the node hasn't been visited
		if (!node_was_visited) {

			//mark this node as visited
			visited_nodes.push_back(node_to_check);

			//add each node in the map for this node
			for_each(this->digraph[node_to_check].cbegin(), this->digraph[node_to_check].cend(),
				[&node_q, &to, &path_exists_flag](std::pair<int, double> to_pair) {
											int node = to_pair.first;
											node_q.push(node); 
											if (node == to) { path_exists_flag = true; }
										 });

		}
	}

	//return whether da to-node were reached by the BFS
	return (path_exists_flag);
	return 0;

}

//inputs: a path
//outputs: whether the path is valid
//details: makes sure evey edge in the path actually exists
bool weighted_digraph::is_path_valid(const list<int> & path) const {

	//itrs point to start node and end node for each edge
	std::list<int>::const_iterator from_itr = path.cbegin();
	std::list<int>::const_iterator to_itr = path.cbegin();
	to_itr++;

	//look at every edge and make sure its in the graph
	for (; to_itr != path.cend(); to_itr++, from_itr++) {
		if (this->digraph[*from_itr].find(*to_itr) == digraph[*from_itr].cend()) {
			return false;
		}
	}

	return true;
}

//inputs: node to go from, node to go to
//outputs: shortest path between the two 
//details: via dijkstra's algorithm
list<int> weighted_digraph::find_minimum_weighted_path(int from, int to) const {
	list<int> path;
	std::set<std::pair<double, int>> node_q; //heap for min weight
	node_q.insert({ 0.0 , from }); //insert first node with a weight 0

	//fill the heap with all other nodes, weight initialize to inf
	for (int i = 0; i < this->numVertices; i++) {
		if (i != from) {
			node_q.insert({ std::numeric_limits<int>::max() ,i });
		}
	}

	//save previous node in the path for each node
	std::vector<int> prev_node (numVertices, numVertices + 1); 

	//mark whether each node has been popped
	std::vector<bool> is_popped(numVertices, false);

	//store shortest lengths
	std::vector<double> lengths(numVertices, std::numeric_limits<double>::max());

	//while not all the nodes have been visited
	while (std::accumulate(is_popped.cbegin(), is_popped.cend(), 0) != is_popped.size() - 1) {

		std::pair<double, int> current_node = *(node_q.begin()); //get node w/ smallest path weight

		if (current_node.second == to) { break; }

		//if the node hasn't already been visited
		if (!is_popped[current_node.second]) {

			//adjust paths that are shorted by being routed through this node
			auto check_new_path = [&](const std::pair<int, double> mapa)
			{
				if (!is_popped[mapa.first] && current_node.first + mapa.second < lengths[mapa.first]) {
					prev_node[mapa.first] = current_node.second;
					node_q.insert({ current_node.first + mapa.second, mapa.first });
					lengths[mapa.first] = current_node.first + mapa.second;
				}
			};

			//adjust paths for every path in the current node's map
			std::for_each(this->digraph[current_node.second].cbegin(), this->digraph[current_node.second].cend(),
				check_new_path);

		}

		is_popped[current_node.second] = true;
		node_q.erase(node_q.begin());

	}

	path.push_front(to); //add the last node to the path

	//fill out the rest of the path
	dijkstra_helper(from, to, path, prev_node);

	return path;
}

//inputs: node to go from, node to go to, path, and list of previous nodes for each node's shortest path
//outputs: none
//side effects: recursively adds nodes to the front of path, until the path is complete
void weighted_digraph::dijkstra_helper(int& from, int to, std::list<int> & path,
	std::vector<int>& prev_node) const {

	//until the starting node is reached, add the previous node from the shortest path to the path
	if (!(from == to)) {
		path.push_front(prev_node[to]);
		dijkstra_helper(from, prev_node[to], path, prev_node);
	}

	return;
}
