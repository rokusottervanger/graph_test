#include "graph_test/World.h"
#include <triplet_graph/Graph.h>
#include <triplet_graph/graph_operations.h>
#include <triplet_graph/PathFinder.h>
#include <triplet_graph/Path.h>

#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>

namespace graph_simulator
{

World::World(): nh_("~") {}

void World::addNode(Node& node)
{
    nodes_.push_back(node);
}

// TODO: throw exceptions instead of returning -1's

bool World::configure(tue::Configuration &config)
{ 
    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Load world from a separate file

    std::string filename;
    if ( config.value("filename",filename))
    {
        tue::Configuration world_config;
        world_config.loadFromYAMLFile(filename);

        // Check if this file defines a graph description of the world
        if ( world_config.readGroup("triplets") )
        {
            world_config.endGroup();

            // Load graph config in a graph
            triplet_graph::Graph graph;
            triplet_graph::configure(graph,world_config);
            std::vector<geo::Vec3d> positions(graph.size());

            std::vector<int> source_nodes;
            if ( config.readArray("initial_nodes") )
            {
                while ( config.nextArrayItem() )
                {
                    std::string id;
                    double x,y;
                    config.value("id",id);
                    config.readGroup("position");
                    config.value("x",x);
                    config.value("y",y);
                    config.endGroup();

                    int n = triplet_graph::findNodeByID(graph,id);
                    if ( n > -1 )
                    {
                        positions[n] = geo::Vec3d(x,y,0);
                        source_nodes.push_back(n);
                    }
                    else
                        std::cout << "[WORLD] configure: WARNING: Could not find initial node with id '" << id << "' in graph" << std::endl;

                }
                config.endArray();
                if ( source_nodes.size() < 2 )
                {
                    std::cout << "[WORLD] configure: ERROR: Not enough initial nodes given." << std::endl;
                    return false;
                }
            }

            // Determine parent-child relations between all connected nodes in graph
            triplet_graph::PathFinder pathFinder(graph,source_nodes);
            triplet_graph::Path path;
            pathFinder.findPath(path);
            if ( path.size() < graph.size())
            {
                std::cout << "[WORLD] configure: WARNING: Not all graph nodes are reachable from the given initial nodes" << std::endl;
            }

            // Calculate positions of all connected nodes in graph
            triplet_graph::calculatePositions(graph,positions,path);

            for ( triplet_graph::Path::iterator it = path.begin(); it != path.end(); ++it )
            {
                Node node;
                node.id = (graph.begin() + *it)->id;
                node.position = positions[*it];
                nodes_.push_back(node);
            }
        }
        // TODO: Make it possible to read cartesian description from external file and make sure this does not mean duplicate code with reading from same config
    }
    else if (config.readArray("nodes"))
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
        config.endArray();

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
            }
            else
                std::cout << "[WORLD] Configure: WARNING: Found robot, but not its initial pose" << std::endl;
            config.value("sensor_frame_id", sensor_frame_id_, tue::REQUIRED);
            config.endGroup();
        }
        else
        {
            std::cout << "[WORLD] Configure: No initial pose of robot found, returning false!" << std::endl;
            return false;
        }
    }

    std::cout << "[WORLD] Configure: DEBUG Sim world now contains " << nodes_.size() << " nodes." << std::endl;


    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure teleop listener

    std::string teleop_topic = "";
    config.value("teleop_topic",teleop_topic);
    ros::NodeHandle nh;
    sub_teleop_ = nh.subscribe<geometry_msgs::Twist>(teleop_topic, 1, &World::teleopCallback, this);

    time_ = ros::Time::now();


    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure visualizer

    if ( config.readGroup("vis") )
        visualizer_.configure(config);

    return true;
}

void World::teleopCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    geo::Transform delta(cmd_vel->linear.x,
                         cmd_vel->linear.y,
                         0,0,0,
                         cmd_vel->angular.z);

    robot_pose_ = delta * robot_pose_;
    std::cout << "Robot pose: " << robot_pose_ << std::endl;
    return;
}

void World::step(triplet_graph::Measurement &measurement)
{
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
