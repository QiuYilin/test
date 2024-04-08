cmake_minimum_required(VERSION 3.11)

set(project_name "default_func")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable(dynamic_memory_alloc dynamic_memory_alloc.cpp 
)

add_executable(custom_class custom_class.cpp currency1.hpp currency2.hpp
)

add_executable(recursion recursion.cpp
)

# add_executable(graph graph/graph.hpp graph/main.cpp
# )

add_executable(linearList   test/linearList.cpp
                            data_structure/linearList/vector.hpp
                            data_structure/linearList/list.hpp
                            data_structure/linearList/dsexceptions.hpp

                            data_structure/linearList/chain.hpp
                            data_structure/linearList/circularListWithHeader.hpp
)

add_executable(list data_structure/linearList/list.hpp test/list.cpp
)

add_executable(stack data_structure/linearList/stack.hpp test/stack.cpp
)

# add_executable(chain data_structure/myExceptions.h
#                      data_structure/linearList.h
#                      data_structure/chain.hpp 
#                      data_structure/chainNode.hpp 
#                      test/chain.cpp
# )

add_executable(linkedDigraph data_structure/myExceptions.h
                             data_structure/changeLength1D.h
                             data_structure/arrayListWithIterator.h

                             data_structure/stack.h
                             data_structure/arrayStack.h
                             
                             data_structure/linkedDigraph.h
                                
                                data_structure/graph.h
                                    data_structure/arrayQueue.h
                                        data_structure/queue.h
                                    data_structure/binNode.h
                                    data_structure/edge.h
                                    data_structure/fastUnionFind.h
                                        data_structure/unionFindNode.h
                                    data_structure/minHeap.h
                                        data_structure/minPriorityQueue.h
                                    data_structure/vertexIterator.h
                                    data_structure/weightedEdge.h
                                data_structure/graphChain.h
                                    data_structure/chainWithIterator.h
                                data_structure/unweightedEdge.h
                             test/linkedDigraph.cpp
)

add_executable(matrix        test/matrix.cpp
                             data_structure/matrix/matrix.hpp
)

add_executable(graph        test/graph.cpp
                            data_structure/graph/graph.hpp
)

add_executable(hash        test/hash.cpp
                            data_structure/hash/pairNode.hpp
                            data_structure/hash/dictionary.hpp
                            data_structure/hash/sortedChain.hpp
                            data_structure/hash/hash.hpp
                            data_structure/hash/hashTable.hpp
                            data_structure/hash/hashChains.hpp
)

add_executable(cutRod algorithm/dynamic_programming/cutRod.cpp)


