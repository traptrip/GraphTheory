#include <iostream>
#include "Graph.h"
#include "Graph.cpp"

using namespace std;

/* 1) Ввод/вывод из файла          (readGraph, writeGraph(representation='C/L/E')) (взвеш/невзвеш ор/неор)
 *      - матрица смемжности       (C)
 *      - список вершин            (L)
 *      - список ребер             (E)
 * 2) Добавление/удаление ребер    (addEdge/removeEdge)
 * 3) Изменение веса ребра         (int changeEdge)
 * 4) Преобразование одного представления в другое (6 методов) (transformTo... 3 функции (2 вложенных метода))
 *
 * класс отдельно описание функций отдельно
 * нельзя создавать класс БОГ! - умеет делать все что только можно
 */


int main() {
    string graphPath = "../graph_files/input_1e3_1e5.txt";

    Graph g;
    g.readGraph(graphPath);
    g.transformToAdjMatrix();
    g.transformToAdjList();
    g.transformToListOfEdges();
    g.writeGraph("../graph_files/output11.txt");

    return 0;
}
