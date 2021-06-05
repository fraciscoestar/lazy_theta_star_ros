#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

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
            CreateCube({0,0,0}, 0, _parent);
            CreateCube({2,1,1}, 1, _parent);
            CreateCube({0,6,5}, 2, _parent);
        }

    private:

        void CreateCube(Vectori pos, int iD, physics::WorldPtr _parent)
        {
            sdf::SDF boxSDF;

            string str = 
                "<sdf version='1.4'>\
                   <model name='box'>\
                    <pose>" + to_string(pos.x) + (string)" " + to_string(pos.y) + (string)" " + to_string(pos.z) + (string)" 0 0 0</pose>\
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