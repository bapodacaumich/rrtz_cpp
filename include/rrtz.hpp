#ifndef RRTZ_HPP
#define RRTZ_HPP

#include "node3d_struct.hpp"
#include "obs.hpp"
#include "plane_struct.hpp"
#include "utils.hpp"
#include "vec3_struct.hpp"

#include <vector>

class RRTZ {
    public:
        // public members
        RRTZ();
        RRTZ(vec3 start, vec3 goal, std::vector<OBS> obs, size_t max_nodes, size_t min_iter, size_t max_iter);
        std::vector<vec3> run();

    private:
        // private members
        vec3 start;
        vec3 goal;
        size_t max_nodes;
        size_t min_iter;
        size_t max_iter;
        std::vector<Node3D> tree_nodes;
        std::vector<OBS> obs;
        std::vector<float> costs;

        bool terminate();
        std::vector<vec3> reconstruct_path(size_t parent_idx);
        Plane sample_plane();
        bool extend();
        std::vector<vec3> get_near_nodes(Plane plane);
        void add_node_to_tree(Node3D node);
};

#endif