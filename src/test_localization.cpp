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
    if ( argc < 3 )
    {
        std::cout << "Usage: \n\n    test_localization SIMULATOR_CONFIG_FILE.yaml GRAPH_CONFIG_FILE.yaml" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "test_localization");

    std::cout << "Starting..." << std::endl;

    tue::Configuration sim_config, graph_config;

    std::cout << "Instantiating sim world" << std::endl;

    graph_simulator::World world;

    std::cout << "Instantiating graph" << std::endl;

    triplet_graph::Graph graph;

    std::cout << "Done!" << std::endl;

    std::cout << "Loading configurations" << std::endl;


    // Load configuration files
    // -------------------------

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

    std::cout << "Configuring simulator... " << std::endl;

    if ( !world.configure(sim_config) )
        return 1;
    std::cout << "Done!" << std::endl << std::endl;

    std::cout << "Configuring graph... " << std::endl;

    if (!triplet_graph::configure(graph,graph_config))
        return 1;

    std::cout << "Done!" << std::endl << std::endl;


    // Test findNodeByID
    // -------------------------

    std::cout << "Testing findNodeByID..." << std::endl;

    int n1 = findNodeByID(graph,"n1");
    int n2 = findNodeByID(graph,"n2");
    std::cout << "Looking for nodes 'n1' and 'n2'" << std::endl;
    if ( n1 > -1 && n2 > -1 )
    {
        std::cout << "Found nodes by ID:" << std::endl;
        std::cout << (graph.begin()+n1)->id << std::endl << (graph.begin()+n2)->id << std::endl << std::endl;
    }
    else
    {
        std::cout << "Could not find those nodes. These nodes are available:" << std::endl;
        for ( Graph::const_iterator it = graph.begin(); it != graph.end(); ++it )
            std::cout << it->id << std::endl;
        return 1;
    }

    // Test simulator
    // -------------------------

    std::cout << "Testing simulator..." << std::endl;

    geo::Pose3D initial_pose(0,0,0);
    Measurement measurement;

//    world.setInitialPose(initial_pose);
    world.step(measurement);

    std::cout << "Simulator added " << measurement.points.size() << " points to the measurement: " << std::endl;
    for ( std::vector<geo::Vec3d>::iterator it = measurement.points.begin(); it != measurement.points.end(); ++it )
    {
        std::cout << *it << std::endl;
    }
    return 0;

    AssociatedMeasurement associations;

    // Add first two nodes to associated measurement
    associations.measurement = measurement;
    associations.measurement.points.pop_back();

    associations.nodes.push_back(0);
    associations.nodes.push_back(1);

    std::cout << "Testing associate function with these prior nodes and one additional node to be associated..." << std::endl;

    // Try to associate all nodes in the graph
    associate(graph, measurement, associations, initial_pose, -1);

    // Show which nodes were succesfully associated:
    std::cout << "Managed to associate " << associations.nodes.size() << " nodes:" << std::endl;
    for ( std::vector<int>::iterator it = associations.nodes.begin(); it != associations.nodes.end(); ++it )
    {
        std::cout << "\tNode " << *it << std::endl;
    }

    std::cout << std::endl;

    return 0;
}
