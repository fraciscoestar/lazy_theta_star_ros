#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#define FPS 25.0

std::string modelName = "quadrotor";
std::string uav_home_frame_id;
geometry_msgs::Pose pose;
geometry_msgs::PoseStamped ref_pose;
geometry_msgs::PoseStamped cur_pose;
geometry_msgs::PoseStamped gazebo_pose;
geometry_msgs::TwistStamped ref_vel;
geometry_msgs::TwistStamped cur_vel;

std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
tf2_ros::Buffer tf_buffer_;

void Move();
geometry_msgs::TwistStamped CalculateRefVel(geometry_msgs::PoseStamped _target_pose);
void ModelStateCallback(const gazebo_msgs::ModelStatesConstPtr& _msg);

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;

    ros::Publisher model_state_publisher_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    ros::Subscriber model_state_subscriber_ = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, ModelStateCallback);

    // Get frame prefix from namespace
    std::string ns = ros::this_node::getNamespace();
    uav_home_frame_id = ns + "/base_link" + "/odom";

    ros::Rate rate(FPS);

    gazebo_msgs::ModelState current;
    current.model_name = modelName;

    while (ros::ok()) 
    {
        ref_vel = CalculateRefVel(ref_pose);
        Move();
            
        current.pose = gazebo_pose.pose;
        current.reference_frame = "map";
        model_state_publisher_.publish(current);
            
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void ModelStateCallback(const gazebo_msgs::ModelStatesConstPtr& _msg)
{
    for (size_t i = 0; i < _msg->name.size(); i++)
    {
        if(_msg->name[i] == modelName)
        {
            pose = _msg->pose[i];
            break;
        }
    }
    
}

geometry_msgs::TwistStamped CalculateRefVel(geometry_msgs::PoseStamped _target_pose) 
{
    geometry_msgs::TwistStamped vel;

    double dx = _target_pose.pose.position.x - cur_pose.pose.position.x;
    double dy = _target_pose.pose.position.y - cur_pose.pose.position.y;
    double dz = _target_pose.pose.position.z - cur_pose.pose.position.z;

    double Th = sqrt( dx*dx + dy*dy ) / 2; // /max_horizontal_velocity_
    double Tz = std::abs( dz / 1 ); // /max_vertical_velocity_
    double T = std::max(Th, Tz);

    if ( T < 1/FPS ) {
        T = 1/FPS;
    }

	vel.twist.linear.x = dx / T;
    vel.twist.linear.y = dy / T;
    vel.twist.linear.z = dz / T;


        // if ( std::abs( dx ) < max_position_error_ ) { 
        //     vel.twist.linear.x = 0.0;
        //     cur_pose_.pose.position.x = 0.8*cur_pose_.pose.position.x + 0.2*_target_pose.pose.position.x;
        // }
        // if ( std::abs( dy ) < max_position_error_ ) {
        //     vel.twist.linear.y = 0.0;
        //     cur_pose_.pose.position.y = 0.8*cur_pose_.pose.position.y + 0.2*_target_pose.pose.position.y;
        // }
        // if ( std::abs( dz ) < max_position_error_ ) {
        //     vel.twist.linear.z = 0.0;
        //     cur_pose_.pose.position.z = 0.8*cur_pose_.pose.position.z + 0.2*_target_pose.pose.position.z;
        // }
        // if ( std::abs( dYaw ) < max_orientation_error_ ) {
        //     vel.twist.angular.z = 0.0;
        //     cur_pose_.pose.orientation.x = 0.5*cur_pose_.pose.orientation.x + 0.5*_target_pose.pose.orientation.x;
        //     cur_pose_.pose.orientation.y = 0.5*cur_pose_.pose.orientation.y + 0.5*_target_pose.pose.orientation.y;
        //     cur_pose_.pose.orientation.z = 0.5*cur_pose_.pose.orientation.z + 0.5*_target_pose.pose.orientation.z;
        //     cur_pose_.pose.orientation.w = 0.5*cur_pose_.pose.orientation.w + 0.5*_target_pose.pose.orientation.w;
        // }

    return vel;
}

void Move() 
{
    double dt = 1 / FPS;

    cur_vel.header.frame_id = uav_home_frame_id;
    cur_vel.twist.linear.x = (0.2 * ref_vel.twist.linear.x + 0.8 * cur_vel.twist.linear.x);
    cur_vel.twist.linear.y = (0.2 * ref_vel.twist.linear.y + 0.8 * cur_vel.twist.linear.y);
    cur_vel.twist.linear.z = (0.2 * ref_vel.twist.linear.z + 0.8 * cur_vel.twist.linear.z);

    cur_pose.pose.position.x += dt * cur_vel.twist.linear.x;
    cur_pose.pose.position.y += dt * cur_vel.twist.linear.y;
    cur_pose.pose.position.z += dt * cur_vel.twist.linear.z;

    // Transform to map
    geometry_msgs::TransformStamped transformToGazeboFrame;

    if ( cached_transforms_.find("inv_map") == cached_transforms_.end() ) {
        // inv_map not found in cached_transforms_
        try {  // TODO: This try-catch is repeated several times, make a function?
            transformToGazeboFrame = tf_buffer_.lookupTransform("map", uav_home_frame_id, ros::Time(0), ros::Duration(0.2));
            cached_transforms_["inv_map"] = transformToGazeboFrame; // Save transform in cache
        } catch (tf2::TransformException &ex) {
            ROS_WARN("At line [%d]: %s", __LINE__, ex.what());
            return;
        }
    } else {
        // found in cache
        transformToGazeboFrame = cached_transforms_["inv_map"];
    }
    tf2::doTransform(cur_pose, gazebo_pose, transformToGazeboFrame);
}