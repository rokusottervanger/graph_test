#ifndef GRAPH_TEST_WORLD_H_
#define GRAPH_TEST_WORLD_H_

#include <geolib/datatypes.h>
#include <geolib/Box.h>
#include <tue/config/configuration.h>
#include <graph_map/Measurement.h>
#include <ros/node_handle.h>

// -----------------------------------------------------------------------------------------------
// Ad Hoc simulator
// -----------------------------------------------------------------------------------------------

namespace graph_map
{

struct Object
{
    std::string id;
    geo::Shape shape;
    geo::Pose3D pose;
};

// -----------------------------------------------------------------------------------------------

class World
{
public:
    World(){}

    void addObject(Object object);

    void configure(tue::Configuration &config);

    std::vector<graph_map::Measurement> step();

private:
    std::vector<Object> objects_;

//    ros::NodeHandle nh_;
};

}

#endif
