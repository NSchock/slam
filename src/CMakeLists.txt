add_library(slam SHARED
        backend.cpp
        camera.cpp
        feature_extractor.cpp
        feature_matcher.cpp
        frame.cpp
        frontend.cpp
        image.cpp
        landmark.cpp
        map.cpp
        slam.cpp
        viewer.cpp
)

target_link_libraries(slam ${THIRD_PARTY_LIBS})
