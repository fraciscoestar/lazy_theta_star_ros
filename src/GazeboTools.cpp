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

            //_parent->SetGravity(ignition::math::Vector3d(0,0,0));
            //_parent->EnablePhysicsEngine(false);

            int i = 0;
            for(auto obstacle : obstacles)
            {
                // cout << "Creating obstacle at " << obstacle.x + "X "  << obstacle.y + "Y "  << obstacle.z + "Z " << endl;
                CreateMorph(obstacle, i, _parent);
                i++;
            }

            for(auto pathPoint : path)
            {
                if(pathPoint == path[0]) // Spawn quadrotor at startpoint
                {
                    SpawnQuadrotor(pathPoint, _parent);
                }

                // cout << "Creating waypoint at " << pathPoint.x + "X "  << pathPoint.y + "Y "  << pathPoint.z + "Z " << endl;
                CreateMorph(pathPoint + Vectorf({0.5f, 0.5f, 0.5f}), i, _parent, Vectorf({0.1f,0.3f,0.8f}), "box", {0.25f, 0.25f, 0.25f}, {0.0f,0.1f,0.0f});
                i++;
            }
        }

    private:

        void SpawnQuadrotor(Vectori position, physics::WorldPtr _parent)
        {
            transport::NodePtr node(new transport::Node());
            node->Init(_parent->GetName());

            transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");
            msgs::Factory msg;
            msg.set_sdf_filename("model://quadrotor");

            msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(ignition::math::Vector3d(position.x, position.y, position.z), ignition::math::Quaterniond(0,0,0)));
            factoryPub->Publish(msg);
        }

        void CreateMorph(Vectori pos, int iD, physics::WorldPtr _parent, Vectorf color = {0.75, 0.75, 1}, string morph = "box", Vectorf scale = {1.0f, 1.0f, 1.0f}, Vectorf emissive = {0,0,0})
        {
            sdf::SDF morphSDF;

                    // DEBAJO DE LINK
                    //  <collision name='collision'>\
                    //   <geometry>\
                    //    <" + morph + ">\
                    //     <size>" + to_string(scale.x) + (string)" " + to_string(scale.y) + (string)" " + to_string(scale.z) + "</size>\
                    //    </" + morph + ">\
                    //   </geometry>\
                    //   <surface>\
                    //    <contact>\
                    //     <collide_without_contact>true</collide_without_contact>\
                    //    </contact>\
                    //   </surface>\
                    //  </collision>\

            string str = 
                "<sdf version='1.6'>\
                   <model name='" + morph + "'>\
                    <pose>" + to_string(pos.x) + (string)" " + to_string(pos.y) + (string)" " + to_string(pos.z+0.5f) + (string)" 0 0 0</pose>\
                    <static>true</static>\
                    <link name='link'>\
                     <visual name='visual'>\
                      <geometry>\
                       <" + morph + ">\
                        <size>" + to_string(scale.x) + (string)" " + to_string(scale.y) + (string)" " + to_string(scale.z) + "</size>\
                       </" + morph + ">\
                      </geometry>\
                      <material>\
                       <ambient>" + to_string(color.x) + (string)" " + to_string(color.y) + (string)" " + to_string(color.z) + " 1</ambient>\
                       <diffuse>" + to_string(color.x) + (string)" " + to_string(color.y) + (string)" " + to_string(color.z) + " 1</diffuse>\
                       <specular>0.1 0.1 0.1 1</specular>\
                       <emissive>" + to_string(emissive.x) + (string)" " + to_string(emissive.y) + (string)" " + to_string(emissive.z) + " 0</emissive>\
                      </material>\
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