#include <stdio.h>
#include <iostream>

#include <triplet_graph/Graph.h>
#include <triplet_graph/graph_operations.h>

#include "graph_test/World.h"

int main(int argc, char** argv)
{
    tue::Configuration sim_config, graph_config;

    // Load configuration file
    // -------------------------

    if ( argc < 3 )
    {
        std::cout << "Usage: \n\n    test_triplet_graph SIMULATOR_CONFIG_FILE.yaml TRIPLET_GRAPH_CONFIG_FILE.yaml" << std::endl;
        return 1;
    }

    std::string sim_config_filename = argv[1];
    sim_config.loadFromYAMLFile(sim_config_filename);

    std::string graph_config_filename = argv[2];
    graph_config.loadFromYAMLFile(graph_config_filename);

    if (sim_config.hasError())
    {
        std::cout << std::endl << "Could not load simulator configuration file:" << std::endl << std::endl << sim_config.error() << std::endl;
        return 1;
    }

    if (graph_config.hasError())
    {
        std::cout << std::endl << "Could not load graph configuration file:" << std::endl << std::endl << graph_config.error() << std::endl;
        return 1;
    }

    // Instantiate graph
    triplet_graph::Graph graph;
    graph_map::World world;

    if (!triplet_graph::configure(graph,graph_config))
        return 1;

    world.configure(sim_config);
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
