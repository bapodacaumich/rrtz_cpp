#ifndef UTILS_HPP
#define UTILS_HPP

#include "node3d_struct.hpp"
#include "triangle_struct.hpp"
#include "vec3_struct.hpp"

static vec3* TEMP = new vec3();

bool ray_int_plane(Node3D node, Plane plane, float eps=1e-9f, vec3& intPoint= * TEMP);
bool ray_int_triangle(Node3D node, Triangle tri, float eps=1e-9f, vec3& intPoint= * TEMP);

#endif // UTILS_HPP