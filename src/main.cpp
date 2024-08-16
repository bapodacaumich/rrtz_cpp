#include "limit_struct.hpp"
#include "node3d_struct.hpp"
#include "obs.hpp"
#include "rrtz.hpp"
#include "triangle_struct.hpp"
#include "utils.hpp"

#include <chrono>
#include <iostream>
#include <vector>

int main() {
    std::vector<std::vector<std::vector<float>>> cubeData;
    loadCube(cubeData);
    std::vector<Triangle> triCubeFaces;
    vecToTri(cubeData, triCubeFaces);

    // create obstacle
    OBS obs = OBS(triCubeFaces);
    std::vector<OBS> obsVec = {obs};

    // create RRTZ
    vec3 start = vec3(-2.0f, 0.1f, 0.1f);
    vec3 goal = vec3(2.0f, 0.1f, 0.1f);
    Limit limits = Limit(-5.0f, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f);
    size_t max_nodes = 1000;
    RRTZ rrtz = RRTZ(start, goal, obsVec, limits, max_nodes);

    // run RRTZ
    std::cout << "Running RRTZ..." << std::endl;
    std::vector<vec3> path;

    // Timing
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    start_time = std::chrono::system_clock::now();

    if (rrtz.run(path)) {
        // timing
        end_time = std::chrono::system_clock::now();

        // compute and display the elapsed time
        std::chrono::duration<double> elapsed_seconds = end_time-start_time;
        std::cout << "\nTime elapsed: " << elapsed_seconds.count() << "s" << std::endl;

        // display path information
        std::cout << "Path found:" << std::endl;
        for (size_t i = 0; i < path.size(); i++) {
            std::cout << path[i].x << " " << path[i].y << " " << path[i].z << std::endl;
        }
    }

    return 0;
}