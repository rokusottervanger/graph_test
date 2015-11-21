#include "graph_test/World.h"

#include <visualization_msgs/Marker.h>

namespace graph_simulator
{

void World::addNode(Node node)
{
    nodes_.push_back(node);
}

void World::configure(tue::Configuration &config)
{
    if (config.readArray("nodes"))
    {
        while (config.nextArrayItem())
        {
            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
            // the user to easily enable and disable certain objects with one single flag.
            int enabled = 1;
            if (config.value("enabled", enabled, tue::OPTIONAL) && !enabled)
                continue;

            Node node;

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Load id

            node.id = "";
            if (!config.value("id", node.id))
                continue;

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Load pose

            node.position = geo::Vec3d(0,0,0);
            if (config.readGroup("position", tue::REQUIRED))
            {
                config.value("x", node.position.x);
                config.value("y", node.position.y);
                config.value("z", node.position.z,tue::OPTIONAL);
                config.endGroup();
            }
            else
                continue;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Load robot pose

    if (config.readArray("robot", tue::REQUIRED))
    {
        robot_pose_ = geo::Pose3D(0,0,0);
        if (config.readGroup("initial_pose", tue::REQUIRED))
        {
            config.value("x", robot_pose_.t.x);
            config.value("y", robot_pose_.t.y);
            config.value("z", robot_pose_.t.z,tue::OPTIONAL);

            double r = 0, p = 0, y = 0;
            config.value("r", r, tue::OPTIONAL);
            config.value("p", p, tue::OPTIONAL);
            config.value("y", y, tue::OPTIONAL);
            robot_pose_.setRPY(r,p,y);
            config.endGroup();
        }
        config.value("sensor_frame_id", sensor_frame_id_, tue::REQUIRED);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure visualizer
    visualizer_.configure(config);
}

void World::step(triplet_graph::Measurement &measurement)
{
//    std::cout << "[SIM] Stepping simulator" << std::endl;

    for ( std::vector<Node>::iterator it = nodes_.begin(); it != nodes_.end(); ++it )
    {
        geo::Vec3d pt = robot_pose_.inverse() * it->position;
        measurement.points.push_back(pt);
    }

    measurement.time_stamp = ros::Time::now();
    measurement.frame_id = sensor_frame_id_;

    if ( visualizer_.isConfigured() )
    {
        visualizer_.publish(measurement);
    }

}

}
