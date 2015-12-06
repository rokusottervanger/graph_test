#include <triplet_graph/CornerDetector.h>
#include <triplet_graph/Graph.h>
#include <triplet_graph/graph_operations.h>
#include <triplet_graph/OdomTracker.h>
#include <triplet_graph/Measurement.h>
#include <triplet_graph/Visualizer.h>
#include <triplet_graph/Path.h>

#include "graph_test/World.h"

#include <tue/profiling/timer.h>
#include <tue/config/configuration.h>

int main(int argc, char** argv)
{
    // - - - - - - - - - - - - - - - - - -
    // Initialize

    tue::Configuration config;

    ros::init(argc, argv, "laser_processor");

    // Parse arguments
    if ( argc < 2 )
    {
        std::cout << "Usage: \n\n        laser_processor LASER_PROCESSOR_CONFIG.yaml" << std::endl;
        return 1;
    }

    std::string config_filename = argv[1];
    std::string graph_filename;
    config.loadFromYAMLFile(config_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load laser processor configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    triplet_graph::Graph graph;

    triplet_graph::CornerDetector cornerDetector;
    graph_simulator::World sim_world;

    triplet_graph::OdomTracker odomTracker;
    triplet_graph::Visualizer visualizer;

    geo::Transform tmp_odom = geo::Transform::identity();


    // - - - - - - - - - - - - - - - - - -
    // Configure everything

    int simulate = 1;
    if ( config.readGroup("simulator") && config.value("enabled",simulate,tue::OPTIONAL) && simulate == 1 )
    {
        std::cout << "Configuring simulator..." << std::endl;
        sim_world.configure(config);
        config.endGroup();
        std::cout << "Done!" << std::endl << std::endl;
    }
    else
    {
        std::cout << "Configuring corner detector..." << std::endl;
        cornerDetector.configure(config);
        std::cout << "Done!" << std::endl << std::endl;
    }

    std::cout << "Configuring odom tracker..." << std::endl;
    odomTracker.configure(config);
    std::cout << "Done!" << std::endl << std::endl;

    if ( config.readGroup("visualization") )
    {
        std::cout << "Configuring visualizer..." << std::endl;
        visualizer.configure(config);
        config.endGroup();
        std::cout << "Done!" << std::endl << std::endl;
    }

    config.value("graph_filename",graph_filename);

    ros::Rate loop_rate(15);

    int target_node = -1;
    int loop = 0;

    while (ros::ok())
    {

        // - - - - - - - - - - - - - - - - - -
        // Start loop timer

        std::cout << "Starting timer" << std::endl;
        tue::Timer timer;
        timer.start();
        std::cout << "Done" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Instantiate stuff

        triplet_graph::Measurement measurement;
        triplet_graph::Measurement unassociated_points;
        triplet_graph::AssociatedMeasurement associations;
        geo::Transform delta = geo::Transform::identity();
        triplet_graph::Path path;


        // - - - - - - - - - - - - - - - - - -
        // Find corners

        std::cout << "Detecting corners" << std::endl;
        if (simulate)
            sim_world.step(measurement);
        else
            cornerDetector.process(measurement);
        std::cout << measurement.points.size() << " corners detected" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Get odom data

        std::cout << "Getting odom delta" << std::endl;
        odomTracker.getDelta(delta,measurement.time_stamp);
        tmp_odom = tmp_odom * delta;
        std::cout << "Done" << std::endl << std::endl;
//        std::cout << "Got odom delta: " << delta << std::endl;
//        std::cout << "Current tmp_odom: " << tmp_odom << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Associate

        std::cout << "Trying to associate..." << std::endl;
        triplet_graph::associate( graph, measurement, associations, unassociated_points, delta, target_node, path );
        std::cout << "Associated " << associations.nodes.size() << " nodes" << std::endl << std::endl;
        if ( associations.nodes.size() > 1 )
            tmp_odom = geo::Transform::identity();

        std::cout << "path.size() = " << path.size() << std::endl;
        std::cout << "graph.size() = " << graph.size() << std::endl;

        // TODO: Make function to easily visualize entire graph

        // - - - - - - - - - - - - - - - - - -
        // Update graph

        // Updates existing edges and adds edges between measured points
//        std::cout << "Updating graph..." << std::endl;
//        triplet_graph::updateGraph( graph, associations );
//        std::cout << "Done!" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Extend graph

//        int graph_size = graph.size();

        if ( loop < 3 && associations.measurement.points.size() < 2 || associations.measurement.points.size() > 1 )
        {
            std::cout << "Extending graph with " << unassociated_points.points.size() << " nodes..." << std::endl;

            triplet_graph::extendGraph( graph, unassociated_points, associations );

            std::cout << "Done!" << std::endl;
            loop ++;
        }


        // - - - - - - - - - - - - - - - - - -
        // Visualize graph

        // Calculate positions again to visualize them in rviz:
//        std::vector<geo::Vec3d> positions(graph.size());
//        for ( int i = 0; i < associations.nodes.size(); ++i )
//            positions[associations.nodes[i]] = associations.measurement.points[i];

//        calculatePositions(graph, positions, path);

//        // Visualize positions
//        triplet_graph::Measurement vis_graph;
//        vis_graph.points = positions;
//        vis_graph.frame_id = measurement.frame_id;
//        vis_graph.time_stamp = measurement.time_stamp;

//        std::cout << "positions vector:" << std::endl;
//        for ( std::vector<geo::Vec3d>::iterator it = positions.begin(); it != positions.end(); ++it )
//            std::cout << *it << std::endl;
//        std::cout << std::endl;
//        std::cout << "Path: " << std::endl;
//        std::cout << path << std::endl;

//        visualizer.publish(vis_graph);


        // - - - - - - - - - - - - - - - - - -
        // Spin ros and sleep

        ros::spinOnce();

        std::cout << "Loop time: " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

        std::cout << std::endl << "----------------------------------------------------------" << std::endl;

        loop_rate.sleep();
    }


    // - - - - - - - - - - - - - - - - - -
    // Save graph

    std::cout << "Writing graph config to disk..." << std::endl;
    triplet_graph::save(graph, graph_filename);
    std::cout << "Saved!" << std::endl;
}

