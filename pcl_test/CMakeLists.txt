cmake_minimum_required(VERSION 3.11)

set(project_name "pcl test")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    add_compile_options(/arch:AVX /arch:AVX2)
else()
    # 32-bit
endif()

find_package(PCL CONFIG REQUIRED)

# add_executable(registration registration.cpp
# )

# target_link_libraries(registration PRIVATE ${PCL_LIBRARIES})

add_executable(pcd_read io/pcd_read.cpp
)

add_executable(pcd_write io/pcd_write.cpp
)

add_executable(concatenate_clouds io/concatenate_clouds.cpp
)

add_executable(kdtree_search k-d_tree/kdtree_search.cpp
)

# add_executable(point_cloud_compression octree/point_cloud_compression.cpp
# )

add_executable(octree_search octree/octree_search.cpp
)

add_executable(octree_change_detection octree/octree_change_detection.cpp
)

add_executable(passthrough octree/octree_change_detection.cpp
)

add_executable(voxelgrid filter/voxelgrid.cpp
)

add_executable(covariance_sampling filter/covariance_sampling.cpp
)

add_executable(range_image_creation range_image/range_image_creation.cpp
)

add_executable(iterative_closest_point registration/iterative_closest_point.cpp
)

add_executable(pairwise_incremental_registration registration/pairwise_incremental_registration.cpp
)

add_executable(normal_distributions_transform registration/normal_distributions_transform.cpp
)

add_executable(interactive_icp registration/interactive_icp.cpp
)

add_executable(alignment_prerejective registration/alignment_prerejective.cpp
)

add_executable(approximatevoxelgrid filter/approximatevoxelgrid.cpp
)

add_executable(project_inliers filter/project_inliers.cpp
)

add_executable(extract_indices filter/extract_indices.cpp
)

add_executable(pointcloud2 advanced/pointcloud2.cpp
)

target_link_libraries(pcd_read PRIVATE ${PCL_LIBRARIES})
target_link_libraries(pcd_write PRIVATE ${PCL_LIBRARIES})
target_link_libraries(concatenate_clouds PRIVATE ${PCL_LIBRARIES})
target_link_libraries(kdtree_search PRIVATE ${PCL_LIBRARIES})
# target_link_libraries(point_cloud_compression PRIVATE ${PCL_LIBRARIES})
target_link_libraries(octree_search PRIVATE ${PCL_LIBRARIES})
target_link_libraries(octree_change_detection PRIVATE ${PCL_LIBRARIES})
target_link_libraries(passthrough PRIVATE ${PCL_LIBRARIES})
target_link_libraries(voxelgrid PRIVATE ${PCL_LIBRARIES})
target_link_libraries(covariance_sampling PRIVATE ${PCL_LIBRARIES})
target_link_libraries(range_image_creation PRIVATE ${PCL_LIBRARIES})
target_link_libraries(iterative_closest_point PRIVATE ${PCL_LIBRARIES})
target_link_libraries(pairwise_incremental_registration PRIVATE ${PCL_LIBRARIES})
target_link_libraries(normal_distributions_transform PRIVATE ${PCL_LIBRARIES})
target_link_libraries(interactive_icp PRIVATE ${PCL_LIBRARIES})
target_link_libraries(alignment_prerejective PRIVATE ${PCL_LIBRARIES})
target_link_libraries(approximatevoxelgrid PRIVATE ${PCL_LIBRARIES})
target_link_libraries(project_inliers PRIVATE ${PCL_LIBRARIES})
target_link_libraries(extract_indices PRIVATE ${PCL_LIBRARIES})
target_link_libraries(pointcloud2 PRIVATE ${PCL_LIBRARIES})