find_package(Boost)
find_package(yaml-cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

include(FetchContent)
fetchcontent_declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
fetchcontent_makeavailable(json)

include_directories(
    include
)

add_library(daa_tracker
    include/data_structures/tree.h
    include/multi_hyp_tracker/MultiHypTracker.h
    include/multi_hyp_tracker/State.h
    include/multi_hyp_tracker/observables.h
    include/utils/FixedEstimator.h
    include/utils/HistogramEstimator.h
    include/utils/KalmanFilter.h
    include/utils/tracker_config.h
    src/multi_hyp_tracker/MultiHypTracker.cpp
    src/multi_hyp_tracker/State.cpp
    src/utils/FixedEstimator.cpp
    src/utils/HistogramEstimator.cpp
    src/utils/KalmanFilter.cpp
    src/utils/estimator_utils.cpp
    src/utils/math.cpp
    src/utils/tracker_config.cpp
)

add_subdirectory(src/cliquer)
add_subdirectory(src/problib)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

target_include_directories(daa_tracker INTERFACE
${CMAKE_CURRENT_SOURCE_DIR}/include
# ${glog_INCLUDE_DIRS}
)

target_link_libraries(daa_tracker
${YAML_CPP_LIBRARIES}
${Boost_LIBRARIES}
${OpenCV_LIBRARIES}
cliquer
problib
nlohmann_json::nlohmann_json
${glog_LIBRARIES}
${glog_LDFLAGS}
)
