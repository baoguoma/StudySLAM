add_library( studyslam SHARED 
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp)

target_link_libraries( studyslam  ${THIRD_PARTY_LIBS} )