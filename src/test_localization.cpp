#include <iostream>
#include <graph_map/Graph.h>
#include <graph_map/Measurement.h>
#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include "graph_test/World.h"

using namespace graph_map;

int main(int argc, char** argv)
{
    tue::Configuration config;
    graph_map::World world = graph_map::World();
    graph_map::Graph graph = graph_map::Graph();


    // Load configuration file
    // -------------------------

    if ( argc < 1 )
    {
        std::cout << "Please provide simulator configuration file as input" << std::endl;
        return 1;
    }

    std::string yaml_filename = argv[1];
    config.loadFromYAMLFile(yaml_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    // Configure sim and graph
    // -------------------------

    world.configure(config);
    graph.configure(config);

    // Test findNodeByID
    // -------------------------

    graph_map::Node* n1 = graph.findNodeByID("box1");
    graph_map::Node* n2 = graph.findNodeByID("box2");
    graph_map::Path path = graph.Dijkstra(n1,n2);

    while (true)
    {
        graph_map::Measurements measurements = world.step();

        graph.update(measurements);

        usleep(0.2e6); // 5Hz
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
    //  - Split up config for graph from that of sim

    std::cout << "Hello world" << std::endl;

    return 0;
}
