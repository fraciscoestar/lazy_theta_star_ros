#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include "lazy_theta_star/map_msg.h"

#include <Utility.hpp>

#include <string>

using namespace std;
using Vectori = Vector<int>;

namespace gazebo
{
    class GazeboTools : public WorldPlugin
    {
    public:

        void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            int argc = 1;
            char *argv[] = {"gazebo"};

            ros::init(argc, argv, "gazebo_subscriber");
            ros::NodeHandle nh;

            ros::Subscriber subs = nh.subscribe<lazy_theta_star::map_msg>("map_topic", 1, MapCallback);

            for (size_t i = 0; i < numObstacles; i++)
            {
                CreateCube(obstacles[i], i, _parent);
            }

            // CreateCube({0,0,0}, 0, _parent);
            // CreateCube({2,1,1}, 1, _parent);
            // CreateCube({0,6,5}, 2, _parent);
        }

    private:

        static vector<Vectori> obstacles;
        static vector<Vectori> path;
        static int numObstacles;
        static int pathSize;

        static void MapCallback(const lazy_theta_star::map_msg::ConstPtr& msg)
        {
            numObstacles = msg->numObstacles;
            pathSize = msg->pathSize;

            for (size_t i = 0; i < msg->numObstacles; i++)
            {
                obstacles.push_back({(int)msg->obstacles[i].x, (int)msg->obstacles[i].y, (int)msg->obstacles[i].z});
            }

            for (size_t i = 0; i < msg->pathSize; i++)
            {
                path.push_back({(int)msg->path[i].x, (int)msg->path[i].y, (int)msg->path[i].z});
            }
            
        }

        void CreateCube(Vectori pos, int iD, physics::WorldPtr _parent)
        {
            sdf::SDF boxSDF;

            string str = 
                "<sdf version='1.4'>\
                   <model name='box'>\
                    <pose>" + to_string(pos.x) + (string)" " + to_string(pos.y) + (string)" " + to_string(pos.z+0.5f) + (string)" 0 0 0</pose>\
                    <static>true</static>\
                    <link name='link'>\
                     <collision name='collision'>\
                      <geometry>\
                       <box>\
                        <size>1 1 1</size>\
                       </box>\
                      </geometry>\
                     </collision>\
                     <visual name='visual'>\
                      <geometry>\
                       <box>\
                        <size>1 1 1</size>\
                       </box>\
                      </geometry>\
                     </visual>\
                    </link>\
                   </model>\
                  </sdf>";


            boxSDF.SetFromString(str);

            sdf::ElementPtr model = boxSDF.Root()->GetElement("model");
            model->GetAttribute("name")->SetFromString(to_string(iD));
            _parent->InsertModelSDF(boxSDF);
        }

    };

    GZ_REGISTER_WORLD_PLUGIN(GazeboTools);
}