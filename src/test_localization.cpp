#include <iostream>
#include <graph_map/Graph.h>
#include <graph_map/Measurement.h>
#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include "graph_test/World.h"

using namespace graph_map;

int main(int argc, char** argv)
{
    tue::Configuration sim_config, graph_config;
    graph_map::World world = graph_map::World();
    graph_map::Graph graph = graph_map::Graph();


    // Load configuration file
    // -------------------------

    if ( argc < 3 )
    {
        std::cout << "Usage: \n\n    test_localization SIMULATOR_CONFIG_FILE.yaml GRAPH_CONFIG_FILE.yaml" << std::endl;
        return 1;
    }

    std::string sim_config_filename = argv[1];
    sim_config.loadFromYAMLFile(sim_config_filename);

    std::string graph_config_filename = argv[2];
    graph_config.loadFromYAMLFile(graph_config_filename);

    if (sim_config.hasError())
    {
        std::cout << std::endl << "Could not load configuration file:" << std::endl << std::endl << sim_config.error() << std::endl;
        return 1;
    }

    if (graph_config.hasError())
    {
        std::cout << std::endl << "Could not load configuration file:" << std::endl << std::endl << graph_config.error() << std::endl;
        return 1;
    }


    // Configure sim and graph
    // -------------------------

    world.configure(sim_config);
    if (!graph.configure(graph_config))
        return 1;


    // Test findNodeByID
    // -------------------------

    graph_map::Node* n1 = graph.findNodeByID("box1");
    graph_map::Node* n2 = graph.findNodeByID("box2");
    std::cout << "Found nodes by ID:" << std::endl;
    std::cout << n1->id << std::endl << n2->id << std::endl;

    // Test Dijkstra
    // -------------------------

    graph_map::Path path = graph.Dijkstra(n1,n2);
    std::cout << "Ran Dijkstra with result:" << std::endl;

    std::cout << path.toString() << std::endl;

    std::cout << "Starting main loop" << std::endl;


    // Main Loop
    // -------------------------

    while (true)
    {
        graph_map::Measurements measurements = world.step();

        graph.update(measurements);

        usleep(0.2e6); // Not 5 Hz!!! todo: make this really 0.2s instead of 0.2s+calculation time...
    }

    // Give initial guess for robot pose (add robot to graph)

    // Step:
    //  - Run simulator (gives object poses for now)
    //      * Generate measurements based on objects in sim world and simulated robot pose
    //  - Run graph update
    //      * Add new objects
    //      * Improve existing relations
    //      * Add new relations

    // Generate measurements based on simulator state
    // Measurement result can be object pose (?)

    // TODO:
    //  - Visualization of sim world (ground truth)
    //  - Visualization of best estimate (graph)

    std::cout << "Hello world" << std::endl;

    return 0;
}
