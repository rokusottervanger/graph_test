#include "graph_test/World.h"

#include <visualization_msgs/MarkerArray.h>

namespace graph_map
{

//World::World(): nh_("~")
//{
//    nh_.advertise<visualization_msgs::MarkerArray>("graph_test/sim/viz",1,true);
//}

void World::addObject(Object object)
{
    objects_.push_back(object);
}

void World::configure(tue::Configuration &config)
{
    if (config.readArray("objects"))
    {
        while (config.nextArrayItem())
        {
            Object obj;

            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
            // the user to easily enable and disable certain objects with one single flag.
            int enabled;
            if (config.value("enabled", enabled, tue::OPTIONAL) && !enabled)
                continue;

            std::string id = "";
            if (!config.value("id", id))
                continue;


            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Load pose

            geo::Pose3D pose = geo::Pose3D::identity();
            if (config.readGroup("pose", tue::REQUIRED))
            {
                config.value("x", pose.t.x);
                config.value("y", pose.t.y);
                config.value("z", pose.t.z);

                double roll = 0, pitch = 0, yaw = 0;
                config.value("roll", roll, tue::OPTIONAL);
                config.value("pitch", pitch, tue::OPTIONAL);
                config.value("yaw", yaw, tue::OPTIONAL);
                pose.R.setRPY(roll, pitch, yaw);

                config.endGroup();
            }
            else
                continue;

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Add shape

            std::string type;
            if (config.value("type", type))
            {
                if (type == "amigo")
                {
                    std::cout << "TODO: Load amigo into simulator object" << std::endl;
                    // Load amigo into object
                }
                else
                {
                    std::cout << "Object '" << id << "'' has unknown object type '" << type << "'" << std::endl;
                }
            }
            else if (config.readArray("shape",tue::OPTIONAL))
            {
                while (config.nextArrayItem())
                {
                    if (config.readGroup("box",tue::OPTIONAL))
                    {
                        geo::Vector3 min, max;

                        if (config.readGroup("min"))
                        {
                            config.value("x", min.x);
                            config.value("y", min.y);
                            config.value("z", min.z);
                            config.endGroup();
                        }

                        if (config.readGroup("max"))
                        {
                            config.value("x", max.x);
                            config.value("y", max.y);
                            config.value("z", max.z);
                            config.endGroup();
                        }

                        geo::Box box(min,max);

                        obj.shape = box;
                        std::cout << "Set box as shape of entity '" << id << "'" << std::endl;

                        config.endGroup();
                    }
                    else
                    {
                        std::cout << "Shape definition of object with id '" << id << "' unknown. Not adding a shape to the entity." << std::endl;
                    }
                }

                // todo: add features based on geometry described in config

                // - - - - - - - - - - - - - - - - - - - - - - - -
                // Add object

                obj.id = id;
                obj.pose = pose;
                addObject(obj);

                std::cout << "[SIM] Added object: id = '" << id << "', pose = " << pose << std::endl;
                config.endArray();
            }
        }
    }
}

Measurements World::step()
{
    std::cout << "[SIM] Stepping simulator" << std::endl;



    Measurements measurements;

    return measurements;
}

}
