#pragma once

#include <AStarPathfinding.hpp>
#include <Utility.hpp>

using namespace std;
using Vectori = Vector<int>;
using Vectorf = Vector<float>;

// Este adaptador es para una grid 3D y 2D.
class AStarAdaptor : public AStarPathfinding::AStarAdaptor
{
public:
    using NodeId = AStarPathfinding::NodeId;
    using Cost = AStarPathfinding::Cost;

    // Para invocar el adaptador.
    AStarAdaptor(const Vectori &mapSize, const function<bool(const Vectori &)> &mIsTraversable) : mMapSize(mapSize), mIsTraversable(mIsTraversable) {}

    // Devuelve la cantidad de nodos del mapa.
    virtual size_t GetNodeCount() const override
    {
        return mMapSize.x * mMapSize.y * mMapSize.z;
    }

    // Devuelve la distancia entre dos nodos.
    virtual Cost Distance(const NodeId n1, const NodeId n2) const override
    {
        return Dist((Vectorf)IdToPos(n1), (Vectorf)IdToPos(n2));
    }

    virtual bool IsObstacle(const NodeId id) const override
    {
        return mIsTraversable(IdToPos(id));
    }

    // Devuelve un vector con todos los vecinos y los costes de ir a cada uno de ellos.
    virtual vector<pair<NodeId, Cost>> GetNodeNeighbors(const NodeId id) const override
    {
        auto pos = IdToPos(id);
        const Cost cost = 1;
        vector<pair<NodeId, Cost>> neighbours;

        if (pos.x != 0 && mIsTraversable({pos.x - 1, pos.y, pos.z}))
            neighbours.push_back({PosToId({pos.x - 1, pos.y, pos.z}), cost});

        if (pos.y != 0 && mIsTraversable({pos.x, pos.y - 1, pos.z}))
            neighbours.push_back({PosToId({pos.x, pos.y - 1, pos.z}), cost});

        if (pos.z != 0 && mIsTraversable({pos.x, pos.y, pos.z - 1}))
            neighbours.push_back({PosToId({pos.x, pos.y, pos.z - 1}), cost});

        if (pos.x != mMapSize.x - 1 && mIsTraversable({pos.x + 1, pos.y, pos.z}))
            neighbours.push_back({PosToId({pos.x + 1, pos.y, pos.z}), cost});

        if (pos.y != mMapSize.y - 1 && mIsTraversable({pos.x, pos.y + 1, pos.z}))
            neighbours.push_back({PosToId({pos.x, pos.y + 1, pos.z}), cost});

        if (pos.z != mMapSize.z - 1 && mIsTraversable({pos.x, pos.y, pos.z + 1}))
            neighbours.push_back({PosToId({pos.x, pos.y, pos.z + 1}), cost});

        return neighbours;
    }

    // Función para convertir de posición a ID.
    NodeId PosToId(const Vectori &pos) const
    {
        // Pos es del tipo {x,y,z}.
        // Id es un único entero unsigned.

        return pos.y * mMapSize.x + pos.x + pos.z * (mMapSize.x * mMapSize.y);
    }

    // Función para convertir de ID a posición.
    Vectori IdToPos(const NodeId id) const
    {
        // Pos es del tipo {x,y,z}.
        // Id es un único entero unsigned.

        int zTerm = (int)id / (mMapSize.x * mMapSize.y);       // Pos. en Z.
        int correctFactor = zTerm * (mMapSize.x * mMapSize.y); // Término para corregir índice 2D.
        return {((int)id - correctFactor) % mMapSize.x, ((int)id - correctFactor) / mMapSize.x, zTerm};
    }

    bool IsTraversable(const Vectori &pos) const
    {
        return mIsTraversable(pos);
    }

private:
    static constexpr const float EPSILON = 0.001f;

    const Vectori mMapSize;
    const function<bool(const Vectori &)> mIsTraversable;

    float Dist(Vectorf v1, Vectorf v2) const
    {
        Vectorf v(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }
};