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
    if ( argc < 2 )
    {
        std::cout << "Usage: \n\n    test_triplet_graph TRIPLET_GRAPH_CONFIG_FILE.yaml" << std::endl;
        return 1;
    }

    std::string graph_config_filename = argv[1];
    graph_config.loadFromYAMLFile(graph_config_filename);
    std::string graph_output_filename = graph_config_filename.substr(0,graph_config_filename.size()-5).append("_output.yaml");
    std::cout << "Reading " << graph_config_filename << std::endl;
    std::cout << "and writing to " << graph_output_filename << std::endl;

    if (graph_config.hasError())
    {
        std::cout << std::endl << "Could not load graph configuration file:" << std::endl << std::endl << graph_config.error() << std::endl;
        return 1;
    }

    // Instantiate graph
    triplet_graph::Graph graph;

    std::cout << "Configuring graph..." << std::endl;
    // Configure graph
    if (!triplet_graph::configure(graph,graph_config))
    {
        std::cout << "Something went wrong configuring the graph" << std::endl;
        return 1;
    }
    std::cout << "Graph succesfully configured. Saving graph..." << std::endl;

    triplet_graph::save(graph,graph_output_filename);

    std::cout << "Graph saved. Check the output!" << std::endl;
}
