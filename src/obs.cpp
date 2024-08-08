#include "obs.hpp"
#include "node3d_struct.hpp"
#include "triangle_struct.hpp"
#include "utils.hpp"
#include "vec3_struct.hpp"

#include <vector>

OBS::OBS() {
    this->faces = std::vector<Triangle>();
    this->n_faces = 0;
}
OBS::OBS(std::vector<Triangle> faces) {
    this->faces = faces;
    this->n_faces = faces.size();
}
size_t OBS::get_n_faces() {
    return this->n_faces;
}
bool OBS::collision(Node3D node) {
    for (size_t i; i < this->n_faces; i++) {
        if (ray_int_triangle(node, this->faces[i])) { return true; }
    }
    return false;
}