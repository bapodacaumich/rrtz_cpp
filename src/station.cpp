#include "limit_struct.hpp"
#include "obs.hpp"
#include "rrtz.hpp"
#include "station.hpp"
#include "utils.hpp"
#include "vec3_struct.hpp"

#include <iostream>

bool solveStation(vec3 start, vec3 goal, std::vector<vec3>& path, size_t max_nodes) {
    // print out the start, goal, and max_nodes
    std::cout << "Solving station: start=" << start.to_string() << " goal=" << goal.to_string() << " max_nodes=" << max_nodes << std::endl;

    Limit limits = { -10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f };
    std::vector<OBS> obsVec;
    loadStationOBS(obsVec);

    RRTZ rrtz = RRTZ(start, goal, obsVec, limits, max_nodes);
    if (!rrtz.run(path)) {
        std::cout << "Failed to find a path." << std::endl;
        return false;
    } else {
        std::cout << "Found path:" << std::endl;
        for (size_t i = 0; i < path.size(); i++) {
            std::cout << path[i].to_string() << std::endl;
        }
        return true;
    }
}