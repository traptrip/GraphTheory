//
// Created by Andrey on 27.02.2021.
//

#include "Graph.h"



// HELP METHODS
void DFS(int u, int p, int step, std::vector<char> &used, std::vector<std::set<int>> &edges,
         std::vector<int> &enter, std::vector<int> &ret, std::vector<std::pair<int, int>> &bridges)
{
    used[u] = true;
    enter[u] = ret[u] = step++;
    for (auto v : edges[u])
    {
        v--;
        if (v == p) continue;
        if (used[v])
            ret[u] = std::min(ret[u], enter[v]);
        else
        {
            DFS(v, u, step, used, edges, enter, ret, bridges);
            ret[u] = std::min(ret[u], ret[v]);
            if (ret[v] > enter[u])
                bridges.emplace_back(u, v);
        }
    }
}


DSU::DSU(int numNodes)
{
    this->numNodes = numNodes;
    parent = std::vector<int>(numNodes);
    rank = std::vector<int>(numNodes, 0);

    for (int i = 0; i < numNodes; i++)
        parent[i] = i;
}


int DSU::find(int x)
{
    if (parent[x] != x)
        parent[x] = find(parent[x]);
    return parent[x];
}


void DSU::unite(int x, int y)
{
    x = find(x);
    y = find(y);

    if (x != y)
    {
        if (rank[x] >= rank[y])
            parent[y] = x;
        else
            parent[x] = y;
        if (rank[x] == rank[y])
            rank[y]++;
    }
}


int DSU::getNumTrees()
{
    std::set<int> counter;
    for (int i = 0; i < numNodes; i++)
        counter.insert(find(i));
    return counter.size();
}


///////////////////////////////////////////////////////////////////////
// MAIN METHODS
Graph::Graph(int numNodes)
{
    graphType = 'L';
    nodesQuantity = numNodes;
    adjacencyListWeighted = std::vector<std::set<std::pair<int, int>>>(numNodes);
    isDirected = false;
    isWeighted = true;
}

Graph::Graph(const std::string& fileName) { this->readGraph(fileName); }

Graph::~Graph()
{
    adjacencyMatrix.clear();
    adjacencyList.clear();
    adjacencyListWeighted.clear();
    listOfEdges.clear();
    listOfEdgesWeighted.clear();
}


// READING METHODS
void Graph::readGraph(const std::string& fileName)
{
    std::string line;
    std::ifstream graphFile(fileName);

    // Read main parameters of the graph
    graphFile >> graphType >> nodesQuantity;
    if (graphType == 'E')
        graphFile >> edgesQuantity;
    graphFile >> isDirected >> isWeighted;
    graphFile.ignore(10, '\n');  // ignore blank line

        // Read graph
    if (graphType == 'C')  // adjacency matrix
        this->readAdjMatrix(graphFile);
    else if (graphType == 'L')  // // adjacency list
        this->readAdjList(graphFile);
    else if (graphType == 'E')  // list of edges
        this->readListOfEdges(graphFile);

    graphFile.close();
}


void Graph::readAdjMatrix(std::ifstream &graphFile)
{
    std::string line;
    while (std::getline(graphFile, line))
    {
        std::istringstream buffer(line);
        std::vector<int> row{std::istream_iterator<int>(buffer), std::istream_iterator<int>()};
        this->adjacencyMatrix.push_back(row);
    }
}


void Graph::readAdjList(std::ifstream &graphFile)
{
    std::string line;
    while (std::getline(graphFile, line))
    {
        if (!isWeighted)
        {
            std::set<int> row;
            std::string num;
            for (char i : line)
            {
                if (i == ' ')
                {
                    row.insert(std::stoi(num));
                    num.clear();
                }
                else
                    num += i;
            }
            row.insert(std::stoi(num));
            this->adjacencyList.push_back(row);
        }
        else
        {
            std::istringstream buffer(line);
            std::vector<int> row((std::istream_iterator<int>(buffer)), std::istream_iterator<int>());
            std::set<std::pair<int, int>> adjNodes;
            std::pair<int, int> nodeAndWeight;

            for (unsigned long i = 1; i < row.size(); i+=2)
            {
                nodeAndWeight.first = row[i-1];
                nodeAndWeight.second = row[i];
                adjNodes.insert(nodeAndWeight);
            }
            this->adjacencyListWeighted.push_back(adjNodes);
        }
    }
}


void Graph::readListOfEdges(std::ifstream &graphFile)
{
    std::string line;
    while (std::getline(graphFile, line))
    {
        std::istringstream buffer(line);
        std::vector<int> row((std::istream_iterator<int>(buffer)), std::istream_iterator<int>());
        if (!isWeighted)
            this->listOfEdges.emplace_back(row[0], row[1]);
        else
            this->listOfEdgesWeighted.emplace_back(row[0], row[1], row[2]);
    }
}


// WRITING METHODS
void Graph::writeGraph(const std::string& fileName)
{
    std::ofstream graphFile;
    graphFile.open (fileName);

    // write base params of the graph
    graphFile << graphType << ' ' << nodesQuantity;
    if (graphType == 'E')
        graphFile << ' ' << edgesQuantity;
    graphFile << "\r\n" << isDirected << ' ' << isWeighted << "\r\n";

    // write graph
    if (graphType == 'C')  // adjacency matrix
        this->writeAdjMatrix(graphFile);
    else if (graphType == 'L')  // // adjacency list
        this->writeAdjList(graphFile);
    else if (graphType == 'E')  // list of edges
        this->writeListOfEdges(graphFile);

    graphFile.close();
}

void Graph::writeAdjMatrix(std::ofstream &graphFile)
{
    for (const auto& row: adjacencyMatrix)
    {
        std::string line;
        for (const auto& val: row) line += std::to_string(val) + ' ';
        line[line.length() - 1] = '\r'; line += '\n';
        graphFile << line;
    }
}

void Graph::writeAdjList(std::ofstream &graphFile)
{
    if (!isWeighted)
        for (const auto& row: adjacencyList)
        {
            std::string line;
            for (auto val : row) line += std::to_string(val) + ' ';
            line[line.length() - 1] = '\r'; line += '\n';
            graphFile << line;
        }
    else
        for (const auto& row: adjacencyListWeighted)
        {
            std::string line;
            for (auto valAndWeight : row)
                line += std::to_string(valAndWeight.first) + ' ' + std::to_string(valAndWeight.second) + ' ';
            line[line.length() - 1] = '\r'; line += '\n';
            graphFile << line;
        }
}

void Graph::writeListOfEdges(std::ofstream &graphFile)
{
    if (!isWeighted)
        for (const auto& edge: listOfEdges)
            graphFile << std::to_string(edge.first) << ' ' << std::to_string(edge.second) << "\r\n";
    else
        for (const auto& edge: listOfEdgesWeighted)
            graphFile << std::to_string(std::get<0>(edge)) << ' '
                      << std::to_string(std::get<1>(edge)) << ' '
                      << std::to_string(std::get<2>(edge)) << "\r\n";
}


// CHANGING METHODS
void Graph::addEdge(int from, int to, int weight)
{
    int from_idx = from - 1, to_idx = to - 1;
    if (!adjacencyMatrix.empty())
        adjacencyMatrix[from_idx][to_idx] = weight;
    else if (!adjacencyList.empty())
    {
        adjacencyList[from_idx].insert(to);
        if (!isDirected)
            adjacencyList[to_idx].insert(from);
    }
    else if (!adjacencyListWeighted.empty())
    {
        adjacencyListWeighted[from_idx].insert(std::make_pair(to, weight));
        if (!isDirected)
            adjacencyListWeighted[to_idx].insert(std::make_pair(from, weight));
    }
    else if (!listOfEdges.empty())
        listOfEdges.emplace_back(from, to);
    else if (!listOfEdgesWeighted.empty())
        listOfEdgesWeighted.emplace_back(from, to, weight);
}


void Graph::removeEdge(int from, int to)
{
    int from_idx = from - 1, to_idx = to - 1;
    if (!adjacencyMatrix.empty())
        adjacencyMatrix[from_idx][to_idx] = 0;

    else if (!adjacencyList.empty())
    {
        adjacencyList[from_idx].erase(to);
        if (!isDirected)
            adjacencyList[to_idx].erase(from);
    }

    else if (!adjacencyListWeighted.empty())
    {
        for (auto node: adjacencyListWeighted[from_idx])
            if (node.first == to)
                adjacencyListWeighted[from_idx].erase(node);
        if (!isDirected)
            for (auto node: adjacencyListWeighted[to_idx])
                if (node.first == from)
                    adjacencyListWeighted[to_idx].erase(node);
    }

    else if (!listOfEdges.empty())
    {
        for (auto it = listOfEdges.begin(); it != listOfEdges.end(); ++it)
        {
            auto edge = *it;
            if (isDirected && edge == std::make_pair(from, to))
                    listOfEdges.erase(it);
            else // isn't Directed
                if (edge == std::make_pair(from, to) || edge == std::make_pair(to, from))
                listOfEdges.erase(it);
        }
    }

    else if (!listOfEdgesWeighted.empty())
    {
        for (auto it = listOfEdgesWeighted.begin(); it != listOfEdgesWeighted.end(); ++it)
        {
            auto edge = *it;
            if (isDirected && std::get<0>(edge) == from && std::get<1>(edge) == to)
                    listOfEdgesWeighted.erase(it);
            else if ((std::get<0>(edge) == from && std::get<1>(edge) == to) ||
                (std::get<0>(edge) == to && std::get<1>(edge) == from))
                    listOfEdgesWeighted.erase(it);
        }
    }
}

int Graph::changeEdge(int from, int to, int newWeight)
{
    // count oldWeight
    int from_idx = from - 1, to_idx = to - 1;
    int oldWeight = 0;
    if (!adjacencyMatrix.empty())
    {
        oldWeight = adjacencyMatrix[from_idx][to_idx];
        if (newWeight > 1) isWeighted = true;
    }
    else if (!adjacencyListWeighted.empty())
    {
        for (auto node: adjacencyListWeighted[from_idx])
            if (node.first == to)
            {
                oldWeight = node.second;
                break;
            }
    }
    else if (!listOfEdgesWeighted.empty())
    {
        for (auto edge : listOfEdgesWeighted)
            if (std::get<0>(edge) == from && std::get<1>(edge) == to)
            {
                oldWeight = std::get<2>(edge);
                break;
            }
    }
    //change edge weight
    this->removeEdge(from, to);
    this->addEdge(from, to, newWeight);
    return oldWeight;
}


// TRANSFORM METHODS
void Graph::transformToAdjList()
{
    if (!adjacencyMatrix.empty())
    {
        adjMatrixToAdjList();
        adjacencyMatrix.clear();
    }
    else if (!(listOfEdges.empty() && listOfEdgesWeighted.empty()))
    {
        listOfEdgesToAdjList();
        listOfEdges.clear();
        listOfEdgesWeighted.clear();
    }
    graphType = 'L';
}

void Graph::transformToAdjMatrix()
{
    if (!(adjacencyList.empty() && adjacencyListWeighted.empty()))
    {
        adjListToAdjMatrix();
        adjacencyList.clear();
        adjacencyListWeighted.clear();
    }
    else if (!(listOfEdges.empty() && listOfEdgesWeighted.empty()))
    {
        listOfEdgesToAdjMatrix();
        listOfEdges.clear();
        listOfEdgesWeighted.clear();
    }
    graphType = 'C';
}

void Graph::transformToListOfEdges()
{
    if (!(adjacencyList.empty() && adjacencyListWeighted.empty()))
    {
        adjListToListOfEdges();
        adjacencyList.clear();
        adjacencyListWeighted.clear();
    }
    else if (!adjacencyMatrix.empty())
    {
        adjMatrixToListOfEdges();
        adjacencyMatrix.clear();
    }
    graphType = 'E';
    edgesQuantity = std::max(listOfEdges.size(), listOfEdgesWeighted.size());
}

void Graph::adjMatrixToAdjList()
{
    if (!isWeighted)
    {
        std::vector<std::set<int>> adjList(nodesQuantity);
        for (unsigned long i = 0; i < adjacencyMatrix.size(); i++)
            for (unsigned long j = 0; j < adjacencyMatrix.size(); j++)
                if (adjacencyMatrix[i][j] > 0)
                    adjList[i].insert(j + 1);
        adjacencyList = std::move(adjList);
    }
    else
    {
        std::vector<std::set<std::pair<int, int>>> adjList(nodesQuantity);
        for (unsigned long i = 0; i < adjacencyMatrix.size(); i++)
            for (unsigned long j = 0; j < adjacencyMatrix.size(); j++)
                if (adjacencyMatrix[i][j] > 0)
                    adjList[i].insert(std::make_pair(j + 1, adjacencyMatrix[i][j]));
        adjacencyListWeighted = std::move(adjList);
    }
}

void Graph::adjMatrixToListOfEdges()
{
    for (int i = 0; i < nodesQuantity; i++)
    {
        if (!isWeighted)
        {
            if (!isDirected)
            {
                for (int j = i; j < nodesQuantity; j++)
                    if (adjacencyMatrix[i][j] > 0)
                        listOfEdges.emplace_back(i+1, j+1);
            }
            else
            {
                for (int j = 0; j < nodesQuantity; j++)
                    if (adjacencyMatrix[i][j] > 0)
                        listOfEdges.emplace_back(i+1, j+1);
            }
        }
        else
        {
            if (!isDirected)
            {
                for (int j = i; j < nodesQuantity; j++)
                    if (adjacencyMatrix[i][j] > 0)
                        listOfEdgesWeighted.emplace_back(i+1, j+1, adjacencyMatrix[i][j]);
            }
            else
            {
                for (int j = 0; j < nodesQuantity; j++)
                    if (adjacencyMatrix[i][j] > 0)
                        listOfEdgesWeighted.emplace_back(i+1, j+1, adjacencyMatrix[i][j]);
            }
        }
    }
}

void Graph::adjListToAdjMatrix()
{
    std::vector<std::vector<int>> adjMatrix(nodesQuantity, std::vector<int> (nodesQuantity, 0));

    if (!isWeighted)
    {
        int i = 0;
        for (const auto& adjNodes: adjacencyList)
        {
            for (auto val: adjNodes)
            {
                adjMatrix[i][val - 1] = 1;
            }
            i++;
        }
        adjacencyMatrix = std::move(adjMatrix);
    }
    else
    {
        int i = 0;
        for (const auto& adjNodes: adjacencyListWeighted)
        {
            for (auto val: adjNodes)
            {
                adjMatrix[i][val.first - 1] = val.second;
            }
            i++;
        }
        adjacencyMatrix = std::move(adjMatrix);
    }
}

void Graph::adjListToListOfEdges()
{
    if (!isWeighted)
    {
        int i = 0;
        for (const auto& adjNodes: adjacencyList)
        {
            for (auto val: adjNodes)
            {
                if (!isDirected && i + 1 < val)
                    listOfEdges.emplace_back(i + 1, val);
                else if (isDirected)
                    listOfEdges.emplace_back(i + 1, val);
            }
            i++;
        }
    }
    else
    {
        int i = 0;
        for (const auto& adjNodes: adjacencyListWeighted)
        {
            for (auto val: adjNodes)
            {
                if (!isDirected && i + 1 < val.first)
                    listOfEdgesWeighted.emplace_back(i + 1, val.first, val.second);
                else if (isDirected)
                    listOfEdgesWeighted.emplace_back(i + 1, val.first, val.second);
            }
            i++;
        }
    }
}

void Graph::listOfEdgesToAdjMatrix()
{
    std::vector<std::vector<int>> adjMatrix(nodesQuantity, std::vector<int> (nodesQuantity, 0));

    if (!isWeighted)
        for (const auto& edge: listOfEdges)
        {
            adjMatrix[edge.first - 1][edge.second - 1] = 1;
            if (!isDirected)
                adjMatrix[edge.second - 1][edge.first - 1] = 1;
        }
    else
        for (const auto& edge: listOfEdgesWeighted)
        {
            adjMatrix[std::get<0>(edge) - 1][std::get<1>(edge) - 1] = std::get<2>(edge);
            if (!isDirected)
                adjMatrix[std::get<1>(edge) - 1][std::get<0>(edge) - 1] = std::get<2>(edge);
        }
    adjacencyMatrix = std::move(adjMatrix);
}

void Graph::listOfEdgesToAdjList()
{
    if (!isWeighted)
    {
        std::vector<std::set<int>> adjList(nodesQuantity);
        for (const auto& edge: listOfEdges)
        {
            adjList[edge.first - 1].insert(edge.second);
            if (!isDirected)
                adjList[edge.second - 1].insert(edge.first);
        }
        adjacencyList = std::move(adjList);
    }
    else
    {
        std::vector<std::set<std::pair<int, int>>> adjList(nodesQuantity);
        for (const auto& edge: listOfEdgesWeighted)
        {
            adjList[std::get<0>(edge) - 1].insert(std::make_pair(std::get<1>(edge), std::get<2>(edge)));
            if (!isDirected)
                adjList[std::get<1>(edge) - 1].insert(std::make_pair(std::get<0>(edge), std::get<2>(edge)));
        }
        adjacencyListWeighted = std::move(adjList);
    }
}


///////////////////////////////////////////////////////////////////////
// minimum spanning tree methods
unsigned long Graph::getRoot(std::vector<bool> usedNodes, unsigned long lastRoot)
{
    for (unsigned long i = lastRoot + 1; i < usedNodes.size(); i++)
        if (!usedNodes[i])
            return i;
    return -1;
}

// Prim's algorithm
Graph Graph::getSpaingTreePrima()
{
    if (nodesQuantity > 0)
    {
        auto previousGraphType = this->graphType;
        this->transformToAdjList();

        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
        std::vector<int> key(nodesQuantity, INT_MAX);
        std::vector<int> parent(nodesQuantity, -1);
        std::vector<bool> used(nodesQuantity, false);
        unsigned long root = 0;

        while (root != -1) {
            pq.push(std::make_pair(0, root));
            key[root] = 0;

            while (!pq.empty())
            {
                int endNode = pq.top().second;
                pq.pop();
                used[endNode] = true;

                for (auto & i : adjacencyListWeighted[endNode])
                {
                    int startNode = i.first - 1;
                    int weight = i.second;

                    if (!used[startNode] && key[startNode] > weight)
                    {
                        key[startNode] = weight;
                        pq.push(std::make_pair(key[startNode], startNode));
                        parent[startNode] = endNode;
                    }
                }
            }
            root = getRoot(used, root);
        }

        if (previousGraphType == 'C') transformToAdjMatrix();
        if (previousGraphType == 'E') transformToListOfEdges();

        Graph minSpanningTree(nodesQuantity);
        for (int i = 0; i < nodesQuantity; i++)
            if (parent[i] != -1)
                minSpanningTree.addEdge(i + 1, parent[i] + 1, key[i]);
        return minSpanningTree;
    }
    Graph minSpanningTree;
    return minSpanningTree;
}

// Kruskal algorithm
Graph Graph::getSpaingTreeKruscal()
{
    if (nodesQuantity > 0)
    {
        std::priority_queue< std::pair<int, std::pair<int, int>>,
        std::vector<std::pair<int, std::pair<int, int>>>,
        std::greater<std::pair<int, std::pair<int, int>>> > pq;

        switch (graphType)
        {
            case 'C':
                for (int i = 0; i < nodesQuantity; i++)
                    for (int j = i + 1; j < nodesQuantity; j++)
                        if (adjacencyMatrix[i][j] != 0)
                            pq.push(make_pair(adjacencyMatrix[i][j], std::make_pair(i, j)));
                break;

            case 'L':
                for (int i = 0; i < nodesQuantity; i++)
                    for (auto j = adjacencyListWeighted[i].begin(); j != adjacencyListWeighted[i].end(); j++)
                        if (j->first - 1 > i)
                            pq.push(make_pair(j->second, std::make_pair(i, j->first - 1)));
                break;

            case 'E':
                for (auto & i : listOfEdgesWeighted)
                    pq.push(make_pair(std::get<2>(i),
                                      std::make_pair(std::get<0>(i) - 1, std::get<1>(i) - 1)));
                break;

            default:
                break;
        }

        Graph minSpanningTree(nodesQuantity);
        DSU disjointSetUnion(nodesQuantity);
        std::pair<int, std::pair<int, int>> edge;
        int weight, startNode, endNode;

        while (!pq.empty())
        {
            edge = pq.top();
            pq.pop();
            weight = edge.first;
            startNode = edge.second.first;
            endNode = edge.second.second;

            if (disjointSetUnion.find(startNode) != disjointSetUnion.find(endNode))
            {
                disjointSetUnion.unite(startNode, endNode);
                minSpanningTree.addEdge(startNode + 1, endNode + 1, weight);
            }
        }
        return minSpanningTree;
    }
    Graph minSpanningTree;
    return minSpanningTree;
}


// Boruvka algorithm
Graph Graph::getSpaingTreeBoruvka()
{
    if (nodesQuantity > 0)
    {
        class Edge
        {
        public:
            int startNode, endNode, edgeWeight;
            Edge() : startNode(-1), endNode(-1), edgeWeight(-1) {}
            Edge(int start, int end, int weight) : startNode(start), endNode(end), edgeWeight(weight) {}
        };

        std::vector<Edge> edges;
        switch (graphType)
        {
            case 'C':
                for (int i = 0; i < nodesQuantity; i++)
                    for (int j = i + 1; j < nodesQuantity; j++)
                        if (adjacencyMatrix[i][j] != 0)
                            edges.emplace_back(i, j, adjacencyMatrix[i][j]);
                break;

            case 'L':
                for (int i = 0; i < nodesQuantity; i++)
                    for (auto j = adjacencyListWeighted[i].begin(); j != adjacencyListWeighted[i].end(); j++)
                        if (j->first - 1 > i)
                            edges.emplace_back(i, j->first - 1, j->second);
                break;

            case 'E':
                for (auto & edge : listOfEdgesWeighted)
                    edges.emplace_back(std::get<0>(edge) - 1, std::get<1>(edge) - 1, std::get<2>(edge));
                break;
        }
        if (graphType != 'E') edgesQuantity = edges.size();

        int startNode, endNode;
        DSU disjointSetUnion(nodesQuantity);

        for (auto & edge : edges)
        {
            startNode = disjointSetUnion.find(edge.startNode);
            endNode = disjointSetUnion.find(edge.endNode);
            if (startNode != endNode)
                disjointSetUnion.unite(startNode, endNode);
        }

        int numberOfTrees = disjointSetUnion.getNumTrees();
        disjointSetUnion = DSU(nodesQuantity);
        std::vector<int> bestEdge(nodesQuantity, -1);
        int treesQuantity = nodesQuantity;
        Graph minSpanningTree(nodesQuantity);

        while (treesQuantity > numberOfTrees)
        {
            for (int i = 0; i < edgesQuantity; i++)
            {
                startNode = disjointSetUnion.find(edges[i].startNode);
                endNode = disjointSetUnion.find(edges[i].endNode);
                if (startNode != endNode)
                {
                    if (bestEdge[startNode] == -1 || edges[bestEdge[startNode]].edgeWeight > edges[i].edgeWeight)
                        bestEdge[startNode] = i;
                    if (bestEdge[endNode] == -1 || edges[bestEdge[endNode]].edgeWeight > edges[i].edgeWeight)
                        bestEdge[endNode] = i;
                }
            }

            for (int & i : bestEdge)
            {
                if (i != -1)
                {
                    startNode = disjointSetUnion.find(edges[i].startNode);
                    endNode = disjointSetUnion.find(edges[i].endNode);
                    if (startNode != endNode)
                    {
                        disjointSetUnion.unite(startNode, endNode);
                        minSpanningTree.addEdge(edges[i].startNode + 1, edges[i].endNode + 1, edges[i].edgeWeight);
                        treesQuantity--;
                    }
                }
            }
            for (int & i : bestEdge) i = -1;
        }
        return minSpanningTree;
    }
    Graph minSpanningTree;
    return minSpanningTree;
}


///////////////////////////////////////////////////////////////////////
// Euler methods
bool Graph::isBridge(int startNode, int endNode, std::vector<std::set<int>> &edges)
{
    std::vector<int> enter(nodesQuantity), ret(nodesQuantity);
    std::vector<char> used(nodesQuantity);
    std::vector<std::pair<int, int>> bridges;
    DFS(startNode, -1, 0, used, edges, enter, ret, bridges);
    for (auto bridge : bridges)
    {
        if (bridge == std::make_pair(startNode, endNode))
            return true;
    }
    return false;
}


// Check if Graph has an Euler path
int Graph::checkEuler(bool & isCycleExist)
{
    transformToAdjList();

    int kOddVertex = 0, resNode = 0;
    for (int i = 0; i < nodesQuantity; ++i)
    {
        if (adjacencyList[i].size() & 1)
        {
            kOddVertex++;
            resNode = i + 1;
        }
    }
    if (kOddVertex <= 2)
    {
        DSU disjointSetUnion(nodesQuantity);
        for (int i = 0; i < nodesQuantity; ++i)
            for (auto edge : adjacencyList[i])
                disjointSetUnion.unite(i, edge - 1);

        std::map<int, int> comp;
        for (int i = 0; i < nodesQuantity; ++i)
            comp[disjointSetUnion.find(i)]++;
        bool isOk = true;
        for (auto item : comp)
        {
            if (item.second > 1)
            {
                if (isOk)
                {
                    if (!resNode)
                        resNode = item.first + 1;
                    isOk = false;
                }
                else
                {
                    resNode = 0;
                    break;
                }
            }
        }
    }
    isCycleExist = (kOddVertex == 0) && resNode;

    return resNode;  // resNode == 0 -> there is no Euler tour
}


// Fleri algorithm
std::vector<int> Graph::getEuleranTourFleri()
{
    transformToAdjList();

    bool isCycleExist;
    int currentNode = checkEuler(isCycleExist) - 1;

    std::vector<std::set<int>> edges(adjacencyList);
    std::vector<int> tour;
    tour.reserve(nodesQuantity);
    tour.push_back(currentNode + 1);

    while (!edges[currentNode].empty())
    {
        int nextNode = -1;
        for (int v : edges[currentNode])
        {
            nextNode = v - 1;
            if (!isBridge(currentNode, nextNode, edges))
                break;
        }
        tour.push_back(nextNode + 1);
        edges[currentNode].erase(nextNode + 1);
        edges[nextNode].erase(currentNode + 1);
        currentNode = nextNode;
    }

    return tour;
}


// Hirholfcher algorithm
std::vector<int> Graph::getEuleranTourEffective()
{
    transformToAdjList();

    std::vector<std::set<int>> edges(adjacencyList);
    std::stack<int> s;
    std::vector<int> tour;
    tour.reserve(nodesQuantity);

    bool isCycleExist;
    int currentNode = checkEuler(isCycleExist) - 1;
    s.push(currentNode);
    while (!s.empty())
    {
        int nextNode = s.top();
        if (!edges[nextNode].empty())
        {
            currentNode = *edges[nextNode].begin() - 1;
            edges[nextNode].erase(currentNode + 1);
            edges[currentNode].erase(nextNode + 1);
            s.push(currentNode);
        }
        else
        {
            tour.push_back(s.top() + 1);
            s.pop();
        }
    }

    return tour;
}
