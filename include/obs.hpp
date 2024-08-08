#ifndef OBS_HPP
#define OBS_HPP

#include "node3d_struct.hpp"
#include "triangle_struct.hpp"
#include <vector>

class OBS { // OBS = Obstacle
    public:
        // public members
        OBS();
        OBS(std::vector<Triangle> faces);
        bool collision(Node3D node);
        size_t get_n_faces();
    private:
        std::vector<Triangle> faces;
        size_t n_faces;
};

#endif // OBS_HPP