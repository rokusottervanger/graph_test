#include "graph_test/World.h"

#include <visualization_msgs/Marker.h>

namespace graph_simulator
{

void World::addNode(Node& node)
{
    nodes_.push_back(node);
}

bool World::configure(tue::Configuration &config)
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

            nodes_.push_back(node);
        }
        std::cout << "[WORLD] Configure: Sim world now contains " << nodes_.size() << " nodes." << std::endl;
        config.endArray();
    }


    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Load robot pose

    if (config.readGroup("robot", tue::REQUIRED))
    {
        robot_pose_ = geo::Pose3D(0,0,0);
        if (config.readGroup("initial_pose", tue::REQUIRED))
        {
            config.value("x", robot_pose_.t.x);
            config.value("y", robot_pose_.t.y);
            config.value("z", robot_pose_.t.z,tue::OPTIONAL);

            double r = 0, p = 0, y = 0;
            config.value("roll", r, tue::OPTIONAL);
            config.value("pitch", p, tue::OPTIONAL);
            config.value("yaw", y, tue::OPTIONAL);
            robot_pose_.setRPY(r,p,y);
            config.endGroup();
            std::cout << "[WORLD] Configure: Added robot at pose (x,y,theta) = ("
                      << robot_pose_.t.x << ","
                      << robot_pose_.t.y << ","
                      << y << ")" << std::endl;
            std::cout << robot_pose_ << std::endl;
        }
        else
            std::cout << "[WORLD] Configure: Found robot, but not its initial pose" << std::endl;
        config.value("sensor_frame_id", sensor_frame_id_, tue::REQUIRED);
        config.endGroup();
    }
    else
    {
        std::cout << "[WORLD] Configure: No initial pose of robot found, returning false!" << std::endl;
        return false;
    }


    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure visualizer

    if ( config.readGroup("vis") )
        visualizer_.configure(config);

    return true;
}


void World::step(triplet_graph::Measurement &measurement)
{
    std::cout << "[WORLD] Step: Stepping simulator containing " << nodes_.size() << " nodes." << std::endl;

    for ( std::vector<Node>::iterator it = nodes_.begin(); it != nodes_.end(); ++it )
    {
        std::cout << "[WORLD] Step: Original point: " << it->position << std::endl;
        std::cout << "[WORLD] Step: Robot pose: " << robot_pose_ << std::endl;
        geo::Vec3d pt = robot_pose_.inverse() * it->position;
        std::cout << "[WORLD] Step: Converted point: " << pt << std::endl;
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
