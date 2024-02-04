cmake_minimum_required(VERSION 3.28)
project("example")

set(CGRAPH_EXAMPLE_LIST
        E01-AutoPilot
        E02-MockGUI
        E03-ThirdFlow
        )
set(CGraph_DIR "C:\\Program Files\\CGraph\\share\\CGraph")
find_package(CGraph REQUIRED)
foreach(example ${CGRAPH_EXAMPLE_LIST})
    add_executable(${example}
            ${example}.cpp
            )
    target_link_libraries(${example} CGraph::CGraph)
endforeach()