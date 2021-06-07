#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>

#include <Utility.hpp>
#include <string>

using namespace std;
using Vectori = Vector<int>;
using Vectorf = Vector<float>;

namespace gazebo
{
    class GazeboTools : public WorldPlugin
    {
    public:

        void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            FilesHandler fh;
            vector<Vectori> obstacles = fh.GetObstaclesFromCSV();
            vector<Vectori> path = fh.GetPathFromCSV();

            int i = 0;
            for(auto obstacle : obstacles)
            {
                CreateMorph(obstacle, i, _parent);
                i++;
            }

            for(auto pathPoint : path)
            {
                CreateMorph(pathPoint, i, _parent, "box", {0.25f, 0.25f, 0.25f});
                i++;
            }
        }

    private:

        void CreateMorph(Vectori pos, int iD, physics::WorldPtr _parent, string morph = "box", Vectorf scale = {1.0f, 1.0f, 1.0f})
        {
            sdf::SDF morphSDF;

            string str = 
                "<sdf version='1.6'>\
                   <model name='" + morph + "'>\
                    <pose>" + to_string(pos.x) + (string)" " + to_string(pos.y) + (string)" " + to_string(pos.z+0.5f) + (string)" 0 0 0</pose>\
                    <static>true</static>\
                    <link name='link'>\
                     <collision name='collision'>\
                      <geometry>\
                       <" + morph + ">\
                        <size>" + to_string(scale.x) + (string)" " + to_string(scale.y) + (string)" " + to_string(scale.z) + "</size>\
                       </" + morph + ">\
                      </geometry>\
                     </collision>\
                     <visual name='visual'>\
                      <geometry>\
                       <" + morph + ">\
                        <size>" + to_string(scale.x) + (string)" " + to_string(scale.y) + (string)" " + to_string(scale.z) + "</size>\
                       </" + morph + ">\
                      </geometry>\
                     </visual>\
                    </link>\
                   </model>\
                  </sdf>";


            morphSDF.SetFromString(str);

            sdf::ElementPtr model = morphSDF.Root()->GetElement("model");
            model->GetAttribute("name")->SetFromString(to_string(iD));
            _parent->InsertModelSDF(morphSDF);
        }

    };

    GZ_REGISTER_WORLD_PLUGIN(GazeboTools);
}