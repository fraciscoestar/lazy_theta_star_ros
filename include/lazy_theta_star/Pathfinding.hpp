#pragma once

#include <vector>
#include <numeric>
#include <algorithm>
#include <functional>

using namespace std;

class Pathfinding
{
public:

    using NodeId = uint32_t;
    using Cost = float; 

    // Se requiere un override de un adaptador, ya que el algoritmo funciona en cualquier tipo de grafo.
    class PathfindingAdaptor
    {
    public:
        friend Pathfinding;

        virtual size_t GetNodeCount() const = 0;
        virtual Cost Distance(const NodeId n1, const NodeId n2) const = 0;
        virtual bool LineOfSight(const NodeId n1, const NodeId n2) const = 0;
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

    Pathfinding(PathfindingAdaptor& adaptor, Cost weight = 1.0f) : adaptor(adaptor), weight(weight)
    {
        GenerateNodes(); // Genera los nodos al invocar el objeto.
    }

    // NOTA: No necesario?
    template<typename DataType>
    vector<DataType> Search(const DataType& start, const DataType& end, function<DataType(const NodeId)> idToData, function<NodeId(const DataType&)> dataToId)
    {
        const auto path = Search(dataToId(start), dataToId(end));
        vector<DataType> finalPath;
        finalPath.reserve(path.size());

        for(const auto id : path)
            finalPath.push_back(idToData(id));

        return finalPath;
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
        while (!openList.empty() && nodes[endId].g > GetMin().f + EPSILON)
        {
            // Sacamos el nodo de menor coste de la lista para expandirlo.
            NodeId currId = GetMin().id;
            PopMin();
            
            // Lazy Theta* asume que siempre hay línea de visión desde el
            // padre de un estado expandido hasta el que lo precede. Cuando
            // expandimos un estado, comprobamos si esto se cumple.
            if(!adaptor.LineOfSight(nodes[currId].parent, currId))
            {
                // Como el nodo no es visible desde el padre, se pone coste
                // infinito.
                nodes[currId].g = INFINITE_COST;

                // Itera a través de los potenciales padres y lo actualiza al padre
                // que da el menor valor de g para el nodo actual.
                for(const auto neighbourInfo : nodes[currId].neighbours)
                {
                    NodeId newParent = neighbourInfo.first; // Obtiene un nuevo posible padre.
                    GenerateState(newParent, endId); // Inicializa el nuevo estado.
                    
                    if(nodes[newParent].list == CLOSED_LIST) // Si estaba en la lista cerrada.
                    {
                        // Calcula el nuevo coste g.
                        Cost newG = nodes[newParent].g + neighbourInfo.second;
                        if(newG < nodes[currId].g) // Si es menor que el coste g del nodo actual.
                        {
                            // Actualiza el coste g del nodo actual y actualiza el padre.
                            nodes[currId].g = newG;
                            nodes[currId].parent = newParent;
                        }
                    }
                }
            }

            // En este punto, el padre tiene visibilidad del estado expandido.

            for(const auto neighbourInfo : nodes[currId].neighbours)
            {
                
                NodeId neighbourId = neighbourInfo.first; // Obtiene un vecino.    
                GenerateState(neighbourId, endId); // Inicializa el vecino.

                NodeId newParent = nodes[currId].parent; // Obtén el padre.

                // Si el vecino no está en la lista cerrada.
                if(nodes[neighbourId].list != CLOSED_LIST)
                {
                    // Calcula el nuevo coste del nodo vecino (a partir del padre obtenido).
                    Cost newG = nodes[newParent].g + adaptor.Distance(newParent, neighbourId);

                    // Si el nuevo coste g es menor que el coste g del vecino actual.
                    if(newG + EPSILON < nodes[neighbourId].g)
                    {
                        nodes[neighbourId].g = newG; // Actualiza el coste g.
                        nodes[neighbourId].parent = newParent; // Actualiza el padre.
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

    PathfindingAdaptor& adaptor;

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