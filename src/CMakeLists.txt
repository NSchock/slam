add_library(slam SHARED
        slam.cpp
        camera.cpp
        frontend.cpp
)

target_link_libraries(slam
        ${THIRD_PARTY_LIBS})
