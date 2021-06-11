#pragma once

#include <vector>
#include <numeric>
#include <algorithm>
#include <functional>

using namespace std;

class AStarPathfinding
{
public:

    using NodeId = uint32_t;
    using Cost = float; 

    // Se requiere un override de un adaptador, ya que el algoritmo funciona en cualquier tipo de grafo.
    class AStarAdaptor
    {
    public:
        friend AStarPathfinding;

        virtual size_t GetNodeCount() const = 0;
        virtual bool IsObstacle(const NodeId id) const = 0;
        virtual Cost Distance(const NodeId n1, const NodeId n2) const = 0;
        virtual vector<pair<NodeId, Cost>> GetNodeNeighbors(const NodeId id) const = 0;
    };

    static constexpr const float EPSILON = 0.00001f;
    static constexpr Cost INFINITE_COST = std::numeric_limits<Cost>::max();

    enum ListType {NO_LIST, OPEN_LIST, CLOSED_LIST}; // Listas en las que puede estar un nodo.

    struct Node
    {
        vector<pair<NodeId, Cost>> neighbours; // Vector de vecinos del nodo.
        uint32_t searchIndex = 0; // La última búsqueda en la que el nodo fue generado. Se usa para decidir si el nodo debe ser reseteado antes de usarlo o no.
        NodeId parent;	// Padre del nodo.
        Cost g;			// Se inicializa a infinito cuando se genera.
        Cost h;		    // Se inicializa a la distancia euclídea al destino cuando se genera.
        ListType list;  // Inicialmente en ninguna lista y puede ser cambiado a la lista abierta o cerrada.
    };

    struct HeapElement
    {
        NodeId id; // Identificación única del nodo.
        Cost g;	// Se usa sólo en caso de empate de coste f.
        Cost f;	// Coste principal.

        // Invertido de manera que el más pequeño está al final del vector.
        bool operator <(const HeapElement& rhs) const
        {
            if(abs(f - rhs.f) < EPSILON) // Si hay empate de coste f:
                return g < rhs.g; // Devuelve true si el coste g del primer operando es menor que el coste g del segundo.

            return f > rhs.f; // Si no hubo empate de coste f, devuelve true si el coste f del primer operando es mayor que el del segundo.
            // Nota: El operando es <, por lo que puede parecer en contra de la lógica.
        }
    };

    AStarPathfinding(AStarAdaptor& adaptor, Cost weight = 1.0f) : adaptor(adaptor), weight(weight)
    {
        GenerateNodes(); // Genera los nodos al invocar el objeto.
    }

    vector<NodeId> Search(const NodeId startId, const NodeId endId)
    {
        openList.clear(); // Limpia la lista abierta.

        currentSearch++; // Aumenta el índice de búsqueda.

        GenerateState(startId, endId); // Inicializa el nodo de inicio.
        GenerateState(endId, endId); // Inicializa el nodo destino.

        nodes[startId].g = 0; // El coste del nodo inicial es cero.
        nodes[startId].parent = startId; // El padre es el mismo.

        AddToOpenList(startId); // Añade el nodo a la lista abierta.

        // Continúa iterando mientras la lista abierta no este vacía y el
        // coste g del nodo destino sea mayor que la f menor de la lista.
        while (!openList.empty())
        {
            // Sacamos el nodo de menor coste de la lista para expandirlo.
            NodeId currId = GetMin().id;
            PopMin();
            
            if(currId == endId)
            {
                break;
            }

            for(const auto neighbourInfo : nodes[currId].neighbours)
            {
                NodeId neighbourId = neighbourInfo.first; // Obtiene un vecino.   

                if(adaptor.IsObstacle(neighbourId))
                    continue;

                GenerateState(neighbourId, endId); // Inicializa el vecino.

                // Si el vecino no está en la lista cerrada.
                if(nodes[neighbourId].list != CLOSED_LIST)
                {
                    // Calcula el nuevo coste del nodo vecino (a partir de la celda actual obtenido).
                    Cost newG = nodes[currId].g + neighbourInfo.second;

                    // Si el nuevo coste g es menor que el coste g del vecino actual.
                    if(newG + EPSILON < nodes[neighbourId].g)
                    {
                        nodes[neighbourId].g = newG; // Actualiza el coste g.
                        nodes[neighbourId].parent = currId; // Actualiza el padre.
                        AddToOpenList(neighbourId); // Añade el vecino a la lista abierta.
                    }
                }
            }
        }

        vector<NodeId> path; // Vector de nodos camino.
        
        // Si ha encontrado camino (coste g de nodo destino no infinito).
        if(nodes[endId].g < INFINITE_COST)
        {
            NodeId currentId = endId; // Empieza el camino por el final.
            while (currentId != startId) // Mientras no se llegue hasta el nodo inicio.
            {
                path.push_back(currentId); // Añade al final del camino el nodo.
                currentId = nodes[currentId].parent; // El siguiente nodo es el padre del actual.
            }
            
            path.push_back(currentId); // Añade el último nodo.
            reverse(path.begin(), path.end()); // Se le da la vuelta al camino para ponerlo en orden correcto.
        }
        else
        {
            path[0] = adaptor.GetNodeCount() + 1;
        }

        return path;
    }

    void GenerateNodes()
    {
        nodes.clear(); // Limpia la lista de nodos.
        nodes.resize(adaptor.GetNodeCount()); // Prealoca el tamaño de la lista.

        NodeId current = 0;
        for(auto& node : nodes) // Recorre todos los nodos y genera la lista de vecinos de cada uno.
            node.neighbours = adaptor.GetNodeNeighbors(current++);
    }

private:

    vector<Node> nodes;
    vector<HeapElement> openList;

    AStarAdaptor& adaptor;

    const Cost weight;

    uint32_t currentSearch = 0;

    void GenerateState(NodeId start, NodeId goal)
    {
        if(nodes[start].searchIndex != currentSearch)
        {
            // Se inicializa el nodo.
            nodes[start].searchIndex = currentSearch;
            nodes[start].h = adaptor.Distance(start, goal) * weight;
            nodes[start].g = INFINITE_COST;
            nodes[start].list = NO_LIST;
        }
    }

    void AddToOpenList(NodeId id)
    {
        // Si ya se encuentra en la lista abierta, lo elimina y lo introduce de forma ordenada.
        if(nodes[id].list == OPEN_LIST)
        {
            auto index = find_if(openList.begin(), openList.end(), [&](const auto& heap){return heap.id == id;});
            auto id = index->id;
            openList.erase(index);
            InsertSorted(openList, {id, nodes[id].g, nodes[id].g + nodes[id].h});       
        }
        else // Si no estaba en la lista abierta, se añade de forma ordenada.
        {
            nodes[id].list = OPEN_LIST;
            InsertSorted(openList, {id, nodes[id].g, nodes[id].g + nodes[id].h});       
        }
    }

    const HeapElement GetMin() const
    {
        return openList.back(); // Devuelve el último elemento del vector, que debería ser el de coste mínimo.
    }

    void PopMin()
    {
        nodes[openList.back().id].list = CLOSED_LIST; // Añade nodo a lista cerrada.
        openList.pop_back(); // Y lo elimina de la lista abierta.
    }

    template<typename T> 
    typename vector<T>::iterator InsertSorted(vector<T> & vec, T const& item)
    {
        return vec.insert(upper_bound(vec.begin(), vec.end(), item), item);
    }

    template<typename T, typename Pred> 
    typename vector<T>::iterator InsertSorted(vector<T> & vec, T const& item, Pred pred)
    {
        return vec.insert(upper_bound(vec.begin(), vec.end(), item, pred), item);
    }

};