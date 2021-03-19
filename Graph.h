//
// Created by Andrey on 27.02.2021.
//

#include <string>
#include <vector>
#include <set>
#include <fstream>
#include "StringUtils.h"

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

    int nodesQuantity, edgesQuantity;
    bool isDirected, isWeighted;
    char graphType;

    void readAdjMatrix(std::ifstream& graphFile);
    void readAdjList(std::ifstream& graphFile);
    void readListOfEdges(std::ifstream& graphFile);

public:
    Graph();
    explicit Graph(const std::string& fileName);

    void readGraph(const std::string& fileName);
    void addEdge(int from, int to, int weight=1);
    void removeEdge(int from, int to);
    int changeEdge(int from, int to, int newWeight);  // return an old weight of an edge
    void transformToAdjList();
    void transformToAdjMatrix();
    void transformToListOfEdges();
    void writeGraph(std::string fileName);
};


#endif //GRAPH_GRAPH_H