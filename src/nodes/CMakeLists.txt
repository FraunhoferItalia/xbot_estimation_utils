find_package(roscpp REQUIRED)
find_package(eigen_conversions REQUIRED)

add_executable(payload_estimation_node
    payload_estimation_node.cpp)

include_directories(
    ${roscpp_INCLUDE_DIRECTORIES}
    ${eigen_conversions_INCLUDE_DIRECTORIES}
)

target_link_libraries(payload_estimation_node
    payload_estimation
    ${roscpp_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    )
