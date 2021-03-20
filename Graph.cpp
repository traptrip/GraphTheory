//
// Created by Andrey on 27.02.2021.
//

#include <sstream>
#include "Graph.h"


Graph::Graph() {}

// READING METHODS

Graph::Graph(const std::string& fileName)
{
    this->readGraph(fileName);
}

void Graph::readGraph(const std::string& fileName)
{
    std::string line;
    std::ifstream graphFile(fileName);

    // Read main parameters of the graph
    graphFile >> graphType >> nodesQuantity;
    if (graphType == 'E')
        graphFile >> edgesQuantity;
    graphFile >> isDirected >> isWeighted;
    graphFile.ignore(1, '\n');  // ignore blank line

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
        {
            std::cout << row[0];
            this->listOfEdgesWeighted.emplace_back(row[0], row[1], row[2]);
        }
    }
}

// WRITING METHOD
void Graph::writeGraph(const std::string& fileName)
{
    std::ofstream graphFile;
    graphFile.open (fileName);

    // write base params of the graph
    graphFile << graphType << ' ' << nodesQuantity;
    if (graphType == 'E')
        graphFile << ' ' << edgesQuantity;
    graphFile << '\n' << isDirected << ' ' << isWeighted << '\n';

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
        line[line.length() - 1] = '\n';
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
            line[line.length() - 1] = '\n';
            graphFile << line;
        }
    else
        for (const auto& row: adjacencyListWeighted)
        {
            std::string line;
            for (auto valAndWeight : row)
                line += std::to_string(valAndWeight.first) + ' ' + std::to_string(valAndWeight.second) + ' ';
            line[line.length() - 1] = '\n';
            graphFile << line;
        }
}

void Graph::writeListOfEdges(std::ofstream &graphFile)
{
    if (!isWeighted)
        for (const auto& edge: listOfEdges)
            graphFile << std::to_string(edge.first) << ' ' << std::to_string(edge.second) << '\n';
    else
        for (const auto& edge: listOfEdgesWeighted)
            graphFile << std::to_string(std::get<0>(edge)) << ' '
                      << std::to_string(std::get<1>(edge)) << ' '
                      << std::to_string(std::get<2>(edge)) << '\n';
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
    this->removeEdge(from, to);
    this->addEdge(from, to, newWeight);
    return oldWeight;
}
