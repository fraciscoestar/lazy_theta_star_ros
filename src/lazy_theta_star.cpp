#include <iostream>
#include <fstream>
#include <sstream>
#include <tuple>
#include <array>
#include <chrono>

#include <Adaptor.hpp>
#include <AStarAdaptor.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#define FPS 25.0

using namespace std;
using namespace ros;

std::string modelName = "quadrotor";
std::string uav_home_frame_id;
geometry_msgs::Pose pose;
geometry_msgs::PoseStamped ref_pose;
geometry_msgs::PoseStamped cur_pose;
geometry_msgs::PoseStamped gazebo_pose;
geometry_msgs::TwistStamped ref_vel;
geometry_msgs::TwistStamped cur_vel;

std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
//tf2_ros::Buffer tf_buffer_;

void Move();
geometry_msgs::TwistStamped CalculateRefVel(geometry_msgs::PoseStamped _target_pose);
void ModelStateCallback(const gazebo_msgs::ModelStatesConstPtr& _msg);

geometry_msgs::Pose pos;
void UpdatePose(const nav_msgs::Odometry);
bool IsValid(Vectori, Vectori);
float pathLength(std::vector<Vectori>);

Vectori IndexToPos(const int id, Vectori mMapSize)
{
    int zTerm = id % (mMapSize.x * mMapSize.y); // Pos. en Z.
    int correctFactor = zTerm * (mMapSize.x * mMapSize.y); // Término para corregir índice 2D.
    return {(id - correctFactor) % mMapSize.x, (id - correctFactor) / mMapSize.x, zTerm};
}


int main(int argc, char *argv[])
{
    init(argc, argv, "lazy_theta_star");
    NodeHandle nh;

    Subscriber odomSubs = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 10, UpdatePose);

    int mapSizeX = 0;
    int mapSizeY = 0;
    int mapSizeZ = 0;

    FilesHandler fh;

    //std::cout << package::getPath("lazy_theta_star") << std::endl;
    string mapName = fh.RetrieveMapName();

////////////////////////////////////////////////////////////////////////////////
    fstream file(package::getPath("lazy_theta_star") + "/worlds/" + mapName);

    if(!file.is_open())
    {
        printf("Error al abrir el archivo\n");
        exit(-1);
    }

    string sizeStr;
    getline(file, sizeStr); // Descarta la primera línea.
    std::stringstream sstream(sizeStr);

    for (size_t i = 0; i < 3; i++) // Obtiene tamaño de mapa.
    {
        std::string val;
        std::getline(sstream, val, ',');

        std::stringstream convertor(val);

        if(i==0)
            convertor >> mapSizeX; 
        else if(i==1)
            convertor >> mapSizeY; 
        else if(i==2)
            convertor >> mapSizeZ; 
    } 

    Vectori mapSize = {mapSizeX, mapSizeY, mapSizeZ};
    array<array<array<char, 100>, 100>, 50> map;

    for (int depth = 0; depth < mapSizeZ; ++depth)
    {
        if(depth != 0) // No es el primer nivel.
        {
            string str;
            getline(file, str); // Descarta la línea vacía.
        }

        for(int row = 0; row < mapSizeY; ++row)
        {
            string line;
            getline(file, line);
            if ( !file.good() ) 
                break;

            stringstream iss(line);  

            for (int col = 0; col < mapSizeX; ++col)
            {
                std::string val;
                std::getline(iss, val, ',');
                if ( !iss.good() ) 
                    break;

                std::stringstream convertor(val);
                convertor >> map[col][row][depth];
                map[col][row][depth] -= '0';
            }
        }      
    }
    
////////////////////////////////////////////////////////////////////////////////

    uint8_t nLines = (mapSizeX/10 > 0) ? 2 : 1;

    std::cout << std::endl;

    for (size_t k = 0; k < mapSizeZ; k++)
    {
        std::cout << "Z level " << k << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        if(nLines > 1)
        {
            std::cout << std::endl << "\t";

            for (size_t i = 0; i < mapSizeX; i++)
            {
                if(i<10)
                {
                    std::cout << " ";
                }
                else
                {
                    std::cout << i/10;
                }
                
            }
        }

        std::cout << std::endl << "\t";
        
        for (size_t i = 0, j = 0; i < mapSizeX; i++, j++)
        {
            if(j == 10)
                j = 0;

            std::cout << j;
        }

        std::cout << std::endl << std::endl;
        

        for (size_t i = 0; i < mapSizeY; i++)
        {
            //std::cout << to_string(i);
            printf("%ld\t", i);

            for (size_t j = 0; j < mapSizeX; j++)
            {
                if(map[j][i][0] == 1)
                    printf("%c", '#');
                else if(map[j][i][k] == 0)
                    printf("%c", ' ');
                else
                    printf("%c", map[j][i][k]);
            }
            printf("\n");
        }

        std::cout << std::endl << "----------------------------------------" << std::endl << std::endl;
    }


    Vectori startPoint = {-1,-1,-1};
    Vectori endPoint = {-1,-1,-1};
    bool valid = false;

    while (!valid)
    {
        std::cout << "Introduzca origen:" << std::endl << "x=";
        std::cin >> startPoint.x;
        std::cout << "y=";
        std::cin >> startPoint.y;
        std::cout << "z=";
        std::cin >> startPoint.z;

        bool insideOfMap = IsValid(startPoint, mapSize);
        if(!insideOfMap)
            continue;

        valid = insideOfMap;

        if((bool)map[startPoint.x][startPoint.y][startPoint.z])
        {
            valid = false;
            std::cout << "La posición " << endPoint.x << "X " << endPoint.y << "Y " << endPoint.z << "Z " << "es un obstáculo!" << "" << std::endl;
        }
    }
    
    valid = false;
    while (!valid)
    {
        std::cout << "Introduzca destino:" << std::endl << "x=";
        std::cin >> endPoint.x;
        std::cout << "y=";
        std::cin >> endPoint.y;
        std::cout << "z=";
        std::cin >> endPoint.z;

        bool insideOfMap = IsValid(startPoint, mapSize);
        if(!insideOfMap)
            continue;

        valid = insideOfMap;

        if((bool)map[endPoint.x][endPoint.y][endPoint.z])
        {
            valid = false;
            std::cout << "La posición " << endPoint.x << "X " << endPoint.y << "Y " << endPoint.z << "Z " << "es un obstáculo!" << "" << std::endl;
        }
    }

    Adaptor adaptor({mapSizeX, mapSizeY, mapSizeZ}, [&map](const Vectori& vec){return map[vec.x][vec.y][vec.z] != 1;});
    Pathfinding pathfinding(adaptor, 100.0f);

    AStarAdaptor aStarAdaptor({mapSizeX, mapSizeY, mapSizeZ}, [&map](const Vectori& vec){return map[vec.x][vec.y][vec.z] != 1;});
    AStarPathfinding aStarPathfinding(aStarAdaptor, 100.0f);

    int algo = 1;
    std::cout << "Algoritmo a utilizar:" << std::endl << "[1] Lazy Theta Star" << std::endl << "[2] A Star" << std::endl;
    std::cin >> algo;

    std::chrono::steady_clock::time_point initTime;
    std::vector<Vectori> path;

    if(algo != 1 && algo != 2)
    {
        std::cout << "Opción no válida!" << std::endl;
        exit(-1);
    }
    else if(algo == 1) //////////////////// LAZY THETA STAR ////////////////////
    {
        initTime = std::chrono::steady_clock::now();
        auto nodePath = pathfinding.Search(adaptor.PosToId(startPoint), adaptor.PosToId(endPoint));

        // Convierte los nodos de ID a posición.
        path.reserve(nodePath.size());

        for(const auto id : nodePath)
            path.push_back(adaptor.IdToPos(id));

        if(path.size())
        {
            path.pop_back();
            path.erase(path.begin());
        }

        if(path.empty() && !adaptor.LineOfSight(adaptor.PosToId(startPoint), adaptor.PosToId(endPoint)))
        {
            std::cout << "No se encontró un camino!" << std::endl;
            exit(-1);
        } 
    }
    else //////////////////// A STAR //////////////////// 
    {
        initTime = std::chrono::steady_clock::now();
        auto nodePath = aStarPathfinding.Search(adaptor.PosToId(startPoint), adaptor.PosToId(endPoint));

        // Convierte los nodos de ID a posición.
        path.reserve(nodePath.size());

        for(const auto id : nodePath)
            path.push_back(aStarAdaptor.IdToPos(id));

        if(path.size())
        {
            path.pop_back();
            path.erase(path.begin());
        }

        if(!IsValid(path[0], mapSize))
        {
            std::cout << "No se encontró un camino!" << std::endl;
            exit(-1);
        } 
    }
    auto endTime = std::chrono::steady_clock::now();

    map[startPoint.x][startPoint.y][startPoint.z] = 'S';
    map[endPoint.x][endPoint.y][endPoint.z] = 'E';

    for (size_t i = 0; i < path.size(); i++)
    {
        map[path[i].x][path[i].y][path[i].z] = i + '1';
    }

    std::cout << std::endl;

    for (size_t k = 0; k < mapSizeZ; k++)
    {
        std::cout << "Z level " << k << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        for (size_t i = 0; i < mapSizeY; i++)
        {
            for (size_t j = 0; j < mapSizeX; j++)
            {
                if(map[j][i][k] == 1)
                    printf("%c", '#');
                else if(map[j][i][k] == 0)
                    printf("%c", ' ');
                else
                    printf("%c", map[j][i][k]);
            }
            printf("\n");
        }

        std::cout << std::endl << "----------------------------------------" << std::endl << std::endl;
    }
    
    std::vector<Vectori> fullPath;

    fullPath.push_back(startPoint);
    for(auto point : path)
    {
        fullPath.push_back(point);
    }
    fullPath.push_back(endPoint);

    std::cout << std::endl << "Finalizado!" << std::endl;
    std::cout << "El camino fue hallado en " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - initTime).count() << "µs." << std::endl;
    std::cout << "Longitud del camino: " + to_string(pathLength(fullPath)) + " metros." << std::endl;

    fh.WritePathToCSV(fullPath);

    std::cout << "Presione una tecla para continuar" << std::endl;
    string s;
    std::cin >> s;

    ros::Publisher model_state_publisher_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    ros::Subscriber model_state_subscriber_ = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, ModelStateCallback);

    // Get frame prefix from namespace
    std::string ns = ros::this_node::getNamespace();
    uav_home_frame_id = ns + "/base_link" + "/odom";

    ros::Rate rate(FPS);

    gazebo_msgs::ModelState current;
    current.model_name = modelName;

    ref_pose.pose = geometry_msgs::Pose();
    ref_pose.pose.position.x = fullPath[0].x;
    ref_pose.pose.position.y = fullPath[0].y;
    ref_pose.pose.position.z = fullPath[0].z + 0.5f;

    gazebo_pose.pose = geometry_msgs::Pose();
    gazebo_pose.pose = ref_pose.pose;

    int i = 0;
    while (ros::ok()) 
    {
        float dx = abs((float)cur_pose.pose.position.x - (float)fullPath[i].x);
        float dy = abs((float)cur_pose.pose.position.y - (float)fullPath[i].y);
        float dz = abs((float)cur_pose.pose.position.z - ((float)fullPath[i].z + 0.5f));

        if(dx < 0.02f && dy < 0.02f && dz < 0.02f)
        {
            std::cout << "Alcanzado waypoint[" + to_string(i+1) + (string)"/" + to_string(fullPath.size()) + "] " << fullPath[i].x << "X " << fullPath[i].y << "Y " << fullPath[i].z << "Z" << std::endl;

            if(i < fullPath.size()-1)
            {
                i++;

                ref_pose.pose.position.x = fullPath[i].x;
                ref_pose.pose.position.y = fullPath[i].y;
                ref_pose.pose.position.z = fullPath[i].z + 0.5f;            
            }
            else
            {
                std::cout << "Destino alcanzado." << std::endl;
                break;
            }
        }

        ref_vel = CalculateRefVel(ref_pose);
        Move();
            
        current.pose = gazebo_pose.pose;
        current.reference_frame = "map";
        model_state_publisher_.publish(current);
            
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "Programa terminado." << std::endl;

    ros::spin();
    
    return 0;
}

float pathLength(std::vector<Vectori> path)
{
    float length = 0;

    for (int i = path.size() - 1; i > 0; i--)
    {
        length += sqrt(pow((path[i].x - path[i-1].x), 2) + pow((path[i].y - path[i-1].y), 2) + pow((path[i].z - path[i-1].z), 2)); 
    }
    
    return length;
}

void UpdatePose(const nav_msgs::Odometry _odom)
{
    pos = _odom.pose.pose;
}

bool IsValid(Vectori v, Vectori limit)
{
    if(v.x<0 || v.x >= limit.x || v.y<0 || v.y >= limit.y || v.z<0 || v.z >= limit.z)
    {
        std::cout << "La posición está fuera de los límites!" << std::endl;
        return false;
    }
    else
        return true;
}

void ModelStateCallback(const gazebo_msgs::ModelStatesConstPtr& _msg)
{
    for (size_t i = 0; i < _msg->name.size(); i++)
    {
        if(_msg->name[i] == modelName)
        {
            cur_pose.pose = _msg->pose[i];
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
    gazebo_pose = cur_pose; //FIX??

    /*if ( cached_transforms_.find("inv_map") == cached_transforms_.end() ) {
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
    tf2::doTransform(cur_pose, gazebo_pose, transformToGazeboFrame);*/
}