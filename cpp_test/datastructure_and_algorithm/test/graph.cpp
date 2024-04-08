#include "../data_structure/graph/graph.hpp"

#include <iostream>

int main() {
  Graph<int> graph;
  auto a = graph.addVertex(1);
  auto b = graph.addVertex(2);
  auto c = graph.addVertex(3);
  auto d = graph.addVertex(4);
  auto e = graph.addVertex(5);
  auto f = graph.addVertex(6);
  auto g = graph.addVertex(7);

  graph.addEdge(a, d);
  graph.addEdge(a, b);
  graph.addEdge(d, g);
  graph.addEdge(g, c);
  graph.addEdge(b, c);
  graph.addEdge(b, e);
  graph.addEdge(c, f);
  graph.addEdge(g, c);

  std::cout << "size " << graph.size() << " " << std::endl;

  std::vector<int> sorted;
  graph.topologicalSort(sorted);

  for (auto& v : sorted) {
    std::cout << v << " ";
  }

  // graph.addEdge(f, a);  // 有环
  // if (graph.topologicalSort(sorted)) {
  //   std::cout << "sorted ";
  // } else {
  //   std::cout << "not sorted ";
  // }

  std::vector<Vertex<int>*> visited;
  graph.depthFirstSearch(d, visited);
  for (auto& v : visited) {
    std::cout << v->value << " ";
  }
  std::cout << std::endl;

  Graph<int> sub_graph = graph.subGraph(b);
  std::vector<int> sub_sorted;
  sub_graph.topologicalSort(sub_sorted);

  for (auto& v : sub_sorted) {
    std::cout << v << " ";
  }
}