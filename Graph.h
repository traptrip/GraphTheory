//
// Created by Andrey on 27.02.2021.
//

#include <string>
#include "fstream"
#include <sstream>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <iterator>
#include <tuple>
#include <climits>
#include <stack>

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H


class Graph
{
private:
    std::vector<std::vector<int>> adjacencyMatrix;
    std::vector<std::set<int>> adjacencyList;
    std::vector<std::set<std::pair<int, int>>> adjacencyListWeighted;
    std::vector<std::pair<int, int>> listOfEdges;
    std::vector<std::tuple<int, int, int>> listOfEdgesWeighted;

    int nodesQuantity{}, edgesQuantity{};
    bool isDirected{}, isWeighted{};
    char graphType{};

    void readAdjMatrix(std::ifstream& graphFile);
    void readAdjList(std::ifstream& graphFile);
    void readListOfEdges(std::ifstream& graphFile);

    void writeAdjMatrix(std::ofstream& graphFile);
    void writeAdjList(std::ofstream& graphFile);
    void writeListOfEdges(std::ofstream& graphFile);

    void adjMatrixToAdjList();
    void adjMatrixToListOfEdges();
    void adjListToAdjMatrix();
    void adjListToListOfEdges();
    void listOfEdgesToAdjMatrix();
    void listOfEdgesToAdjList();

    static unsigned long getRoot(std::vector<bool> usedNodes, unsigned long lastRoot);
    bool isBridge(int startNode, int endNode, std::vector<std::set<int>> &edges);

public:
    explicit Graph(int numNodes = 0);
    explicit Graph(const std::string& fileName);
    ~Graph();

    void readGraph(const std::string& fileName);
    void addEdge(int from, int to, int weight=1);
    void removeEdge(int from, int to);
    int changeEdge(int from, int to, int newWeight);  // return an old weight of an edge
    void transformToAdjList();
    void transformToAdjMatrix();
    void transformToListOfEdges();
    void writeGraph(const std::string& fileName);

    // min spanning tree methods
    Graph getSpaingTreePrima();
    Graph getSpaingTreeKruscal();
    Graph getSpaingTreeBoruvka();

    // euler path / loop
    int checkEuler(bool& circleExist); // Check if Graph has Euler path
    std::vector<int> getEuleranTourFleri();
    std::vector<int> getEuleranTourEffective();
};


// Disjoint Set Union
class DSU
{
private:
    int numNodes;
    std::vector<int> parent, rank;

public:
    DSU(int numNodes);
    int find(int x);
    void unite(int x, int y);
    int getNumTrees();
};


#endif //GRAPH_GRAPH_H
