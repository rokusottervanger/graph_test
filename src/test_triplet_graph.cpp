#include <stdio.h>
#include <iostream>

#include <triplet_graph/Path.h>
#include <triplet_graph/Graph.h>
#include <triplet_graph/graph_operations.h>

#include "graph_test/World.h"

int main(int argc, char** argv)
{
    tue::Configuration graph_config;

    // Parse arguments
    if ( argc < 5 )
    {
        std::cout << "Usage: \n\n    test_triplet_graph TRIPLET_GRAPH_CONFIG_FILE.yaml target_node source_node1 source_node2 [source_node3...n]" << std::endl;
        return 1;
    }

    std::string graph_config_filename = argv[1];
    graph_config.loadFromYAMLFile(graph_config_filename);

    if (graph_config.hasError())
    {
        std::cout << std::endl << "Could not load graph configuration file:" << std::endl << std::endl << graph_config.error() << std::endl;
        return 1;
    }

    // Instantiate graph
    triplet_graph::Graph graph;

    // Configure graph
    if (!triplet_graph::configure(graph,graph_config))
        return 1;

    // Get node index of target node
    int ntarget = triplet_graph::findNodeByID(graph,argv[2]);

    std::vector<int> source_nodes;
    // Get node indices of source nodes
    for ( int i = 3; i < argc; i++ )
    {
        int node_index = triplet_graph::findNodeByID(graph,argv[i]);
        source_nodes.push_back(node_index);
    }

    triplet_graph::Path path;

    double cost = triplet_graph::findPath(graph,source_nodes,ntarget,path);
    std::cout << "Found path: " << std::endl;
    std::cout << path << std::endl;
    std::cout << "With a total cost of " << cost << std::endl;
}

// What should it do?
// -------------------------
//
// Retrieve data from sensor
// ( Obtain feature points from sensor data)
// for point in all feature points:
//      if !associate point with existing node
//          add new node
//      Add edge2s from node to all nodes added or associated in this cycle
//      add edge3s from node to all nodes added or associated in this cycle
