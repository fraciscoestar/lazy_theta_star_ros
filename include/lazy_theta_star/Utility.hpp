#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <tuple>
#include <array>

#include <stdio.h>
// #include <yaml-cpp/yaml.h>
#include <ros/package.h>

template<typename T>
struct Vector
{
    T x,y,z;

    Vector() = default;
    Vector(const Vector<T>&) = default; 
    Vector(const T x, const T y, const T z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    template<typename S>
    Vector(const Vector<S>& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    Vector<T> operator -(const Vector<T>& other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vector<T> operator +(const Vector<T>& other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    bool operator ==(const Vector<T>& other) const
    {
        if(x == other.x && y == other.y && z == other.z)
            return true;
        else
            return false;
    }

    Vector<T> operator *(float k) const
    {
        return {x * k, y * k, z * k};
    }

    Vector<T> operator /(float k) const
    {
        return {x / k, y / k, z / k};
    }
};

class FilesHandler
{
public:

    std::vector<Vector<int>> GetObstaclesFromCSV()
    {
        std::vector<Vector<int>> obstacles;

        std::fstream file(ros::package::getPath("lazy_theta_star") + "/worlds/" + RetrieveMapName());

        if(!file.is_open())
        {
            std::cout << "No se pudo abrir el mapa!" << std::endl;
            exit(-1);
        }

        std::string sizeStr;
        std::getline(file, sizeStr);
        std::stringstream sstream(sizeStr);
        
        for (size_t i = 0; i < 3; i++) // Obtiene tamaño de mapa.
        {
            std::string val;
            std::getline(sstream, val, ',');

            std::stringstream convertor(val);
            convertor >> mapSize[i];
        }       

        int i = 0;
        for (int depth = 0; depth < mapSize[2]; depth++)
        {
            if(depth != 0) // No es el primer nivel.
            {
                std::string str;
                std::getline(file, str); // Descarta la línea vacía.
            }

            for(int row = 0; row < mapSize[1]; ++row)
            {
                std::string line;
                std::getline(file, line);
                if ( !file.good() ) 
                    break;

                std::stringstream iss(line);  

                for (int col = 0; col < mapSize[0]; ++col)
                {
                    char cellValue;
                    std::string val;
                    std::getline(iss, val, ',');
                    if ( !iss.good() ) 
                        break;

                    std::stringstream convertor(val);
                    convertor >> cellValue;
                    
                    if(cellValue == '1')
                    {
                        obstacles.push_back({col, row, depth});
                    }
                }
            }         
        }
        

        return obstacles;
    }

    void WritePathToCSV(std::vector<Vector<int>> path)
    {
        std::string filePath = ros::package::getPath("lazy_theta_star") + "/worlds/path.csv";

        try
        {
            remove(&filePath[0]);
        }
        catch(const std::exception& e)
        {
            std::cout << "El fichero de camino no existe. Creando archivo..." << std::endl;
        }
        
        std::fstream fs;
        fs.open(filePath, std::fstream::out);

        int pathSize = path.size();
        fs << std::to_string(pathSize) << "," << std::endl;

        for(auto point : path)
        {
            fs << std::to_string(point.x) << ", " << std::to_string(point.y) << ", " << std::to_string(point.z) << "," << std::endl;
        }

        fs.close();
    }

    std::vector<Vector<int>> GetPathFromCSV()
    {
        std::vector<Vector<int>> path;
        int pathSize = 0;

        std::fstream file(ros::package::getPath("lazy_theta_star") + "/worlds/" + RetrieveMapName());

        if(!file.is_open())
        {
            std::cout << "No se pudo abrir el camino!" << std::endl;
            exit(-1);
        }

        std::string sizeStr;
        std::getline(file, sizeStr);
        std::stringstream sstream(sizeStr);
        
        std::string val;
        std::getline(sstream, val, ',');

        std::stringstream convertor(val);
        convertor >> pathSize;     

        //path.reserve(pathSize);  

        int i = 0;
        
        for (size_t i = 0; i < pathSize; i++)
        {
            std::string line;
            std::getline(file, line);
            if(!file.good())
                break;

            std::stringstream iss(line);
            Vector<int> point;

            for (size_t j = 0; j < 3; j++)
            {
                std::string val;
                std::getline(iss, val, ',');
                if(!iss.good())
                    break;

                std::stringstream convertor(val);
                //convertor >> val;

                if(j==0)
                {
                    convertor >> point.x;
                    // std::cout << path[i].x << std::endl;
                }
                else if(j==1)
                {
                    convertor >> point.y;
                    // std::cout << path[i].y << std::endl;
                }
                else if(j==2)
                {
                    convertor >> point.z;
                    // std::cout << path[i].z << std::endl;
                }

                
            }

            path.push_back(point);
        }        

        return path;
    }

    // YAML::Node GetConfig()
    // {
    //     return YAML::LoadFile(ros::package::getPath("lazy_theta_star") + "/config/config.yaml");
    // }

    std::string RetrieveMapName()
    {
        return "map_lazy_theta.csv";
        // YAML::Node node = GetConfig();
        // std::string mapName = "";

        // if(node["LTS"]["map"])
        //     mapName = node["LTS"]["map"].as<std::string>();
        // else
        //     std::cout << "Hubo un error leyendo el nombre del mapa!" << std::endl;

        // return mapName;
    }

private:
    std::vector<int> mapSize = {0, 0, 0};

};