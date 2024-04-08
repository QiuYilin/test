#pragma once
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

template <typename T>
struct Vertex {  // 每个顶点的数据和数组邻接表
  T value;
  std::vector<Vertex<T>*> previous;
  std::vector<Vertex<T>*> next;
};

template <typename T>
class Graph {
 private:
  std::vector<Vertex<T>*> _vertices;  // 保存所有顶点的数组

 public:
  Graph() = default;
  ~Graph() {
    for (auto& v : _vertices) {
      delete v;
    }
  }
  Graph(const Graph&) = default;
  Graph& operator=(const Graph&) = default;
  Graph(Graph&&) = default;
  Graph& operator=(Graph&&) = default;

  Vertex<T>* addVertex(T value) {
    _vertices.push_back(new Vertex<T>{value});
    return _vertices.back();
  }

  void addEdge(Vertex<T>* vertex1, Vertex<T>* vertex2) {
    if (existVertex(vertex1) && existVertex(vertex2)) {
      vertex1->next.push_back(vertex2);
      vertex2->previous.push_back(vertex1);
    } else {
      throw std::invalid_argument("vertex not found");
    }
  }

  void deleteEdge(Vertex<T>* vertex1, Vertex<T>* vertex2) {
    if (existVertex(vertex1) && existVertex(vertex2)) {
      auto it = std::find(vertex1->next.begin(), vertex1->next.end(), vertex2);
      if (it != vertex1->next.end()) {
        vertex1->next.erase(it);
      }
      it = std::find(vertex2->previous.begin(), vertex2->previous.end(),
                     vertex1);
      if (it != vertex2->previous.end()) {
        vertex2->previous.erase(it);
      }
    } else {
      throw std::invalid_argument("vertex not found");
    }
  }

  // 拓扑排序 Kahn算法
  bool topologicalSort(std::vector<T>& sorted) {
    std::vector<Vertex<T>*> indegreeZero;  // 存放入度为0的顶点
    std::vector<int> indegree(_vertices.size(), 0);  // 存放每个顶点的入度

    // 计算每个顶点的入度
    for (int i = 0; i < _vertices.size(); i++) {
      indegree[i] = _vertices[i]->previous.size();
      if (indegree[i] == 0) {
        indegreeZero.push_back(_vertices[i]);
      }
    }

    while (!indegreeZero.empty()) {
      // 将入度为0的顶点加入到sorted数组中
      Vertex<T>* v = indegreeZero.back();
      indegreeZero.pop_back();
      sorted.push_back(v->value);

      // 更新与v相邻的顶点的入度
      for (auto& next : v->next) {
        auto it = std::find(_vertices.begin(), _vertices.end(), next);
        if (it != _vertices.end()) {
          int index = std::distance(_vertices.begin(), it);
          indegree[index]--;
          if (indegree[index] == 0) {
            indegreeZero.push_back(next);
          }
        }
      }
    }

    // 如果存在入度不为0的顶点，则说明图中存在环，无法进行拓扑排序
    for (int i = 0; i < _vertices.size(); i++) {
      if (indegree[i] != 0) {
        return false;
      }
    }

    return true;
  }

  int size() { return _vertices.size(); }

  Graph<T>& clone() {
    Graph<T> newGraph;
    std::unordered_map<Vertex<T>*, Vertex<T>*> map;
    for (Vertex<T>* v : _vertices) {
      Vertex<T>* newV = new Vertex<T>;
      newV->value = v->value;
      newGraph._vertices.push_back(newV);
      map[v] = newV;
    }
    for (Vertex<T>* v : _vertices) {
      for (Vertex<T>* next : v->next) {
        map[v]->next.push_back(map[next]);
      }
      for (Vertex<T>* prev : v->previous) {
        map[v]->previous.push_back(map[prev]);
      }
    }
    return newGraph;
  }

  void depthFirstSearch(Vertex<T>* startVertex,
                        std::vector<Vertex<T>*>& vertices) {
    std::unordered_set<Vertex<T>*> visited;  // Used to store visited vertices

    // Call the recursive helper function
    depthFirstSearchHelper(startVertex, visited, vertices);
  }

  Graph<T> subGraph(Vertex<T>* startVertex) {
    std::vector<Vertex<T>*> vertices;
    depthFirstSearch(startVertex, vertices);

    Graph<T> newGraph;
    std::unordered_map<Vertex<T>*, Vertex<T>*> map;

    for (Vertex<T>* v : vertices) {
      Vertex<T>* newV = new Vertex<T>;
      newV->value = v->value;
      newGraph._vertices.push_back(newV);
      map[v] = newV;
    }

    for (Vertex<T>* v : vertices) {
      for (auto& next : v->next) {
        if (map.count(next) > 0) {
          map[v]->next.push_back(map[next]);
        }
      }
      for (auto& prev : v->previous) {
        if (map.count(prev) > 0) {
          map[v]->previous.push_back(map[prev]);
        }
      }
    }

    return newGraph;
  }

 private:
  void depthFirstSearchHelper(Vertex<T>* currentVertex,
                              std::unordered_set<Vertex<T>*>& visited,
                              std::vector<Vertex<T>*>& vertices) {
    // Mark the current vertex as visited
    visited.insert(currentVertex);

    // Add the current vertex to the result vector
    vertices.push_back(currentVertex);

    // Recur for all the adjacent vertices
    for (auto& next : currentVertex->next) {
      if (visited.find(next) ==
          visited.end()) {  // If the adjacent vertex is not visited
        depthFirstSearchHelper(next, visited, vertices);
      }
    }
  }

 private:
  bool existVertex(Vertex<T>* vertex) {
    for (auto& v : _vertices) {
      if (v == vertex) {
        return true;
      }
    }
    return false;
  }
};