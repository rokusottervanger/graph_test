#ifndef GRAPH_TEST_WORLD_H_
#define GRAPH_TEST_WORLD_H_

#include <geolib/datatypes.h>
#include <geolib/Box.h>
#include <tue/config/configuration.h>
#include <triplet_graph/Measurement.h>
#include <ros/node_handle.h>
#include <triplet_graph/Visualizer.h>
#include <geometry_msgs/Twist.h>

// -----------------------------------------------------------------------------------------------
// Ad Hoc simulator
// -----------------------------------------------------------------------------------------------

namespace graph_simulator
{

struct Node
{
    std::string id;
    geo::Vec3d position;
};

// -----------------------------------------------------------------------------------------------

class World
{
public:
    World();

    void addNode(Node&);

    bool configure(tue::Configuration&);

    void setInitialPose(geo::Pose3D& pose) { robot_pose_ = pose; }

    void step(triplet_graph::Measurement&);

private:
    std::vector<Node> nodes_;

    geo::Pose3D robot_pose_;
    std::string sensor_frame_id_;
    triplet_graph::Visualizer visualizer_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_teleop_;
    void teleopCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    ros::Time time_;
};

}

#endif
