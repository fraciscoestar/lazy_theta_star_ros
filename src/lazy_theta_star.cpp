#include <iostream>
#include <fstream>
#include <sstream>
#include <tuple>
#include <array>

#include <Adaptor.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "lazy_theta_star/map_msg.h"

using namespace std;
using namespace ros;

geometry_msgs::Pose pos;
void UpdatePose(const nav_msgs::Odometry _odom);
bool IsValid(Vectori, Vectori);

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

    constexpr int mapSizeX = 20;
    constexpr int mapSizeY = 16;
    constexpr int mapSizeZ = 1;
    Vectori mapSize = {mapSizeX, mapSizeY, mapSizeZ};

    array<array<array<char, mapSizeZ>, mapSizeY>, mapSizeX> map;

    fstream file("/home/fraci/catkin_ws/src/lazy_theta_star/worlds/map.csv");


    if(!file.is_open())
    {
        printf("Error al abrir el archivo\n");
        exit(-1);
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
            convertor >> map[col][row][0];
            map[col][row][0] -= '0';
        }
    }

    Publisher pub = nh.advertise<lazy_theta_star::map_msg>("/map_topic", 1000);
    Rate loop_rate(1);

    lazy_theta_star::map_msg mapMsg;
    mapMsg.numObstacles = 0;
    mapMsg.obstacles.reserve(mapSizeX*mapSizeY*mapSizeZ/2);
    mapMsg.path.reserve(100);
    //mapMsg.mapSize = {mapSizeX, mapSizeY, mapSizeZ};

    int i = 0;
    for (size_t z = 0; z < mapSizeZ; z++)
    {
        for (size_t y = 0; y < mapSizeY; y++)
        {
            for (size_t x = 0; x < mapSizeX; x++)
            {
                if(map[x][y][z] == 1)    
                {
                    mapMsg.obstacles[i].x = x;
                    mapMsg.obstacles[i].y = y;
                    mapMsg.obstacles[i].z = z;
                    i++;
                    mapMsg.numObstacles++;
                }        
            }
                    
        }
            
    }
    

    // int x_s, y_s, z_s, x_e, y_e, z_e;
    Vectori startPoint = {-1,-1,-1};
    Vectori endPoint = {-1,-1,-1};
    bool valid = false;

    while (!valid)
    {
        cout << "Introduzca origen:" << endl << "x=";
        cin >> startPoint.x;
        cout << "y=";
        cin >> startPoint.y;
        cout << "z=";
        cin >> startPoint.z;

        bool insideOfMap = IsValid(startPoint, mapSize);
        valid = insideOfMap;

        if(insideOfMap && (bool)map[startPoint.x][startPoint.y][startPoint.z])
        {
            valid = false;
            cout << "La posición es un obstáculo!" << endl;
        }
    }
    
    valid = false;
    while (!valid)
    {
        cout << "Introduzca destino:" << endl << "x=";
        cin >> endPoint.x;
        cout << "y=";
        cin >> endPoint.y;
        cout << "z=";
        cin >> endPoint.z;

        bool insideOfMap = IsValid(startPoint, mapSize);
        valid = insideOfMap;

        if(insideOfMap && (bool)map[endPoint.x][endPoint.y][endPoint.z])
        {
            valid = false;
            cout << "La posición es un obstáculo!" << endl;
        }
    }

    Adaptor adaptor({mapSizeX, mapSizeY, mapSizeZ}, [&map](const Vectori& vec){return map[vec.x][vec.y][vec.z] != 1;});
    Pathfinding pathfinding(adaptor, 100.0f);

    auto nodePath = pathfinding.Search(adaptor.PosToId(startPoint), adaptor.PosToId(endPoint));

    // Convierte los nodos de ID a posición.
    vector<Vectori> path;
    path.reserve(nodePath.size());

    for(const auto id : nodePath)
        path.push_back(adaptor.IdToPos(id));

    if(path.size())
    {
        path.pop_back();
        path.erase(path.begin());
    }

    map[startPoint.x][startPoint.y][startPoint.z] = 'S';
    map[endPoint.x][endPoint.y][endPoint.z] = 'E';

    mapMsg.pathSize = path.size();

    for (size_t i = 0; i < path.size(); i++)
    {
        map[path[i].x][path[i].y][path[i].z] = i + '1';
        mapMsg.path[i].x = path[i].x;
        mapMsg.path[i].y = path[i].y;
        mapMsg.path[i].z = path[i].z;
    }

    for (size_t i = 0; i < mapSizeY; i++)
    {
        for (size_t j = 0; j < mapSizeX; j++)
        {
            if(map[j][i][0] == 1)
                printf("%c", '#');
            else if(map[j][i][0] == 0)
                printf("%c", ' ');
            else
                printf("%c", map[j][i][0]);
        }
        printf("\n");
    }

    printf("\nFinished!\n");

    while (pub.getNumSubscribers() == 0)
    {
        loop_rate.sleep();
    }
    
    pub.publish(mapMsg);

    spin();
    
    return 1;
}

void UpdatePose(const nav_msgs::Odometry _odom)
{
    pos = _odom.pose.pose;
}

bool IsValid(Vectori v, Vectori limit)
{
    if(v.x<0 || v.x >= limit.x || v.y<0 || v.y >= limit.y || v.z<0 || v.z >= limit.z)
    {
        cout << "La posición está fuera de los límites!" << endl;
    }
}