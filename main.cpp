#include <iostream>
#include "Graph.h"
#include "Graph.cpp"
//#include "StringUtils.h"

using namespace std;

/* 1) Ввод/вывод из файла          (readGraph, writeGraph(representation='C/L/E')) (взвеш/невзвеш ор/неор)
 *      - матрица смемжности       (C)
 *      - список вершин            (L)
 *      - список ребер             (E)
 * 2) Добавление/удаление ребер    (addEdge/removeEdge)
 * 3) Изменение веса ребра         (int changeEdge)
 * 4) Преобразование одного представления в другое (6 методов) (transformTo... 3 функции (2 вложенных метода))
 *
 * должны быть конструкторы для каждого представления и деструктор
 *
 * На что смотрят:
 * класс отдельно описание функций отдельно
 * нельзя класс БОГ! - умеет делать все что только можно
 *
 * main() {
 * Graph g;
 * g.readGraph('путь_до_файла');
 * g.transformTo...()
 * g.writeGraph('путь_до_нового_файла1')
 * g.transformTo...()
 * g.writeGraph('путь_до_нового_файла2')
 * g.transformTo...()
 * g.writeGraph('путь_до_нового_файла3')
 * последнее преобразование должно получить исходный файл (ничего если отсортировано не в том порядке)
 * (смотрится объем файлов)
 */


int main() {
//    string graphPath = "/Users/and/Desktop/3курс/2сем/GraphTheory/Graph/graph_files/input_L1_D_0_W_1.txt";
//    Graph g;
//    g.readGraph(graphPath);
//
//    g.changeEdge(1, 2, 5);
    int nodesQuantity, edgesQuantity;
    bool isDirected, isWeighted;
    string graphType = "C";

    nodesQuantity = 10;
    isDirected = isWeighted = false;

    std::ofstream graphFile;
    graphFile.open ("output_graph.txt");

    graphFile << graphType << ' ' << nodesQuantity << ' ' << edgesQuantity << '\n' << isDirected << ' ' << isWeighted << '\n';
    graphFile.close();

    return 0;
}