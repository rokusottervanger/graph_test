#include "graph_test/World.h"

namespace graph_map
{

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

            std::string id;
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
            // Add object

            obj.id = id;
            obj.pose = pose;
            addObject(obj);

            std::cout << "[SIM] Added object: id = '" << id << "', pose = " << pose << std::endl;

        }

        config.endArray();
    }
}

Measurements World::step()
{
    std::cout << "[SIM] Stepping simulator" << std::endl;

    Measurements measurements;

    return measurements;
}

}
