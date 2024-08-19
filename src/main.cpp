#include "limit_struct.hpp"
#include "node3d_struct.hpp"
#include "obs.hpp"
#include "rrtz.hpp"
#include "station.hpp"
#include "triangle_struct.hpp"
#include "utils.hpp"

#include <chrono>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    // argc is the number of arguments passed to the program
    // argv is an array of strings containing the arguments

    // check for correct number of arguments
    if (argc != 1 && argc != 2) {
        std::cout << "Usage: ./rrtz or ./rrtz max_nodes" << std::endl;
        return 1;
    }

    size_t max_nodes = 500;
    if (argc == 2) {
        std::cout << "Running with max nodes=" << argv[1] << std::endl;
        max_nodes = std::stoi(argv[1]);
    }

    // create start and goal
    vec3 start = vec3(-0.5f, -2.0f, -0.5f);
    vec3 goal = vec3(1.0f, 4.0f, 0.0f);
    std::vector<vec3> path;
    solveStation(start, goal, path, max_nodes);
    return 0;

    // // create obstacles
    // bool cube = false;
    // bool station = true;
    // std::vector<OBS> obsVec;
    // Limit limits;
    // vec3 start;
    // vec3 goal;
    // if (cube) {
    //     // load in cube data and convert to triangles
    //     std::vector<std::vector<std::vector<float>>> cubeData;
    //     loadCube(cubeData);
    //     std::vector<Triangle> triCubeFaces;
    //     vecToTri(cubeData, triCubeFaces);

    //     // create obstacle
    //     OBS obs = OBS(triCubeFaces);
    //     obsVec.push_back(obs);

    //     // create limits
    //     limits.set(-5.0f, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f);
    //     start.set(-2.0f, 0.1f, 0.1f);
    //     goal.set(2.0f, 0.1f, 0.1f);
    // } else if (station) {
    //     // load in station to obsVec
    //     loadStationOBS(obsVec);
    //     limits.set(-10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
    //     start.set(-0.5f, -2.0f, -0.5f);
    //     goal.set(1.0f, 4.0f, 0.0f);
    // }

    // // create RRTZ
    // RRTZ rrtz = RRTZ(start, goal, obsVec, limits, max_nodes);

    // // run RRTZ
    // std::cout << "Running RRTZ..." << std::endl;
    // std::vector<vec3> path;

    // // Timing
    // std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    // start_time = std::chrono::system_clock::now();

    // if (rrtz.run(path)) {
    //     // timing
    //     end_time = std::chrono::system_clock::now();

    //     // compute and display the elapsed time
    //     std::chrono::duration<double> elapsed_seconds = end_time-start_time;
    //     std::cout << "\nTime elapsed: " << elapsed_seconds.count() << "s" << std::endl;

    //     // display path information
    //     std::cout << "Path found:" << std::endl;
    //     for (size_t i = 0; i < path.size(); i++) {
    //         std::cout << path[i].x << " " << path[i].y << " " << path[i].z << std::endl;
    //     }
    // }

    // return 0;
}