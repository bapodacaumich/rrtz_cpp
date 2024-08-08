#include "node3d_struct.hpp"
#include "obs.hpp"
#include "plane_struct.hpp"
#include "rrtz.hpp"
#include "utils.hpp"
#include "vec3_struct.hpp"

#include <vector>

RRTZ::RRTZ() {
    this->start = vec3();
    this->goal = vec3();
    this->max_nodes = 0;
    this->min_iter = 0;
    this->max_iter = 0;
    this->tree_nodes = std::vector<Node3D>();
    this->obs = std::vector<OBS>();
}

RRTZ::RRTZ(vec3 start, vec3 goal, std::vector<OBS> obs, size_t max_nodes, size_t min_iter, size_t max_iter) {
    // save problem config
    this->start = start;
    this->goal = goal;
    this->max_nodes = max_nodes;
    this->min_iter = min_iter;
    this->max_iter = max_iter;

    // initialize tree
    this->tree_nodes = std::vector<Node3D>();
    this->tree_nodes.push_back(Node3D(start, start, 0, 0));

    // get obstacles
    this->obs = obs;
}

std::vector<vec3> RRTZ::run() {
    // initialize best solution
    
}

bool RRTZ::terminate() {}
std::vector<vec3> RRTZ::reconstruct_path(size_t parent_idx) {}
Plane RRTZ::sample_plane() {}
bool RRTZ::extend() {}
std::vector<vec3> RRTZ::get_near_nodes(Plane plane) {}
void RRTZ::add_node_to_tree(Node3D node) {}