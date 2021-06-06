#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <tuple>
#include <array>

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

        std::fstream file("/home/fraci/catkin_ws/src/lazy_theta_star/worlds/map_lazy_theta.csv");

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

private:
    std::vector<int> mapSize = {0, 0, 0};

};