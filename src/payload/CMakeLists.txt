find_package(XBotInterface REQUIRED)

add_library(payload_estimation SHARED
    payload_estimation.cpp)

set_target_properties(payload_estimation PROPERTIES PREFIX "${PROJECT_NAME}_")

target_link_libraries(payload_estimation
    PUBLIC
    kalman
    Eigen3::Eigen
    XBotInterface::XBotInterface
)

target_include_directories(payload_estimation
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS payload_estimation
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib)
