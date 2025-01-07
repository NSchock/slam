add_library(slam SHARED
        camera.cpp
        feature_extractor.cpp
        feature_matcher.cpp
        frame.cpp
        frontend.cpp
        image.cpp
        landmark.cpp
        map.cpp
        slam.cpp
)

target_link_libraries(slam
        ${THIRD_PARTY_LIBS})
