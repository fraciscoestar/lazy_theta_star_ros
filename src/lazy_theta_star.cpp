#include <iostream>
#include <fstream>
#include <sstream>
#include <tuple>
#include <array>

#include <Adaptor.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

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

    int mapSizeX = 0;
    int mapSizeY = 0;
    int mapSizeZ = 0;

    FilesHandler fh;

    //cout << package::getPath("lazy_theta_star") << endl;
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

    // for(size_t t = 0; t < mapSizeX; t++) //FIX
    // {
    //     printf("%ld", t);
    // }
    // printf("\n\n");

    for (size_t i = 0; i < mapSizeY; i++)
    {
        //cout << to_string(i);
        printf("%ld\t", i);

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

    if(path.empty() && !adaptor.LineOfSight(adaptor.PosToId(startPoint), adaptor.PosToId(endPoint)))
    {
        cout << "No se encontró un camino!" << endl;
        exit(-1);
    }

    map[startPoint.x][startPoint.y][startPoint.z] = 'S';
    map[endPoint.x][endPoint.y][endPoint.z] = 'E';

    for (size_t i = 0; i < path.size(); i++)
    {
        map[path[i].x][path[i].y][path[i].z] = i + '1';
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

    
    vector<Vectori> fullPath;

    fullPath.push_back(startPoint);
    for(auto point : path)
    {
        fullPath.push_back(point);
    }
    fullPath.push_back(endPoint);

    fh.WritePathToCSV(fullPath);


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