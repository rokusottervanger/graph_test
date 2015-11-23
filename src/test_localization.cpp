#include <iostream>
#include <triplet_graph/Graph.h>
#include <triplet_graph/graph_operations.h>
#include <triplet_graph/Measurement.h>
#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include "graph_test/World.h"

using namespace triplet_graph;

/*
 * TODOs:

-   Run simulator (gives object poses for now)
    *   Generate measurements based on objects in sim world and
        simulated robot pose (but what should they look like,
        and when should measurements be created? Only when the
        robot looks at the object? When does the robot look at objects?)
-   Run graph update. Based on new measurements:
    *   Add new objects
    *   Improve existing relations
    *   Add new relations
-   Visualization of sim world (ground truth)
-   Visualization of best estimate (graph)
*/


int main(int argc, char** argv)
{
    tue::Configuration sim_config, graph_config;
    graph_simulator::World world;
    Graph graph;


    // Load configuration files
    // -------------------------

    if ( argc < 3 )
    {
        std::cout << "Usage: \n\n    test_localization SIMULATOR_CONFIG_FILE.yaml GRAPH_CONFIG_FILE.yaml" << std::endl;
        return 1;
    }

    std::cout << "Loading simulator config file... ";

    std::string sim_config_filename = argv[1];
    sim_config.loadFromYAMLFile(sim_config_filename);

    std::cout << "Done!" << std::endl;

    std::cout << "Loading graph config file... ";

    std::cout << "Done!" << std::endl;

    std::string graph_config_filename = argv[2];
    graph_config.loadFromYAMLFile(graph_config_filename);

    if (sim_config.hasError())
    {
        std::cout << std::endl << "Could not load simulator configuration file:" << std::endl << sim_config.error() << std::endl;
        return 1;
    }

    if (graph_config.hasError())
    {
        std::cout << std::endl << "Could not load graph configuration file:" << std::endl << graph_config.error() << std::endl;
        return 1;
    }

    std::cout << std::endl;


    // Configure sim and graph
    // -------------------------

    std::cout << "Configuring simulator... ";

    world.configure(sim_config);

    std::cout << "Done!" << std::endl;
    std::cout << "Configuring graph... ";

    if (!triplet_graph::configure(graph,graph_config))
        return 1;

    std::cout << "Done!" << std::endl << std::endl;


    // Test findNodeByID
    // -------------------------

    std::cout << "Testing findNodeByID..." << std::endl;

    int n1 = findNodeByID(graph,"box1");
    int n2 = findNodeByID(graph,"box2");
    std::cout << "Looking for nodes 'box1' and 'box2'" << std::endl;
    std::cout << "Found nodes by ID:" << std::endl;
    std::cout << (graph.begin()+n1)->id << std::endl << (graph.begin()+n2)->id << std::endl;


    // Generate initial pose (associated measurement)
    // -------------------------

    std::cout << "Testing associate function..." << std::endl;

    geo::Pose3D initial_pose(0,0,0);
    Measurement measurement;

    world.setInitialPose(initial_pose);
    world.step(measurement);

    AssociatedMeasurement associations;

    for ( std::vector<geo::Vec3d>::iterator it = measurement.points.begin(); it !=  )

    associate(graph, measurement, associations, initial_pose, -1);





    // Main Loop
    // -------------------------

//    while (true)
//    {
//        graph_map::Measurements measurements = world.step();

//        graph.update(measurements);

//        usleep(0.2e6); // Not 5 Hz!!! todo: make this really 0.2s instead of 0.2s+calculation time...
//        // todo: only localize when you have a task, right?
//    }

    return 0;
}
