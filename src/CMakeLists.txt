add_library(slam SHARED
        slam.cpp
        camera.cpp
)

target_link_libraries(slam
        ${THIRD_PARTY_LIBS})
