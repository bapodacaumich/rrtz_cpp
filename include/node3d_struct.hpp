#ifndef NODE3D_STRUCT_HPP
#define NODE3D_STRUCT_HPP

#include "plane_struct.hpp"
#include "vec3_struct.hpp"

#include <vector>
#include <limits>
#include <stddef.h>

struct Node3D {
    vec3 origin;
    vec3 vector;
    vec3 end;
    float cost;
    size_t idx;
    size_t parent_idx;
    Plane plane;
    Node3D() {
        origin = vec3();
        vector = vec3();
        end = vec3();
        cost = std::numeric_limits<float>::infinity();
        idx = -1;
        parent_idx = -1;
        plane = Plane();
    }
    Node3D(vec3 o, vec3 e, size_t i, size_t p) {
        origin = o;
        end = e;
        vector = e - o;
        idx = i;
        parent_idx = p;
    }
};

#endif // NODE3D_STRUCT_HPP