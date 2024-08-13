#include "node3d_struct.hpp"
#include "obs.hpp"
#include "rrtz.hpp"
#include "triangle_struct.hpp"
#include "utils.hpp"

#include <iostream>
#include <vector>

int main(int argc, char* argv[]) {
    // argc is number of args
    // argv is array of args
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
    size_t max_nodes = 10000;
    RRTZ rrtz = RRTZ(start, goal, obsVec, limits, max_nodes);

    // run RRTZ
    std::cout << "Running RRTZ..." << std::endl;
    std::vector<vec3> path;
    if (rrtz.run(path)) {
        std::cout << "Path found:" << std::endl;
        for (size_t i = 0; i < path.size(); i++) {
            std::cout << path[i].x << " " << path[i].y << " " << path[i].z << std::endl;
        }
    }

    return 0;
}