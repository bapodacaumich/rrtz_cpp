#include "triangle_struct.hpp"
#include "utils.hpp"
#include "vec3_struct.hpp"
#include <cmath>

bool ray_int_plane(Node3D node, Plane plane, float eps, vec3& intPoint) {
    vec3 origin_to_point = plane.point - node.origin;
    vec3 end_to_point = node.end - node.origin;
    float origin_dot = origin_to_point.dot(plane.normal);
    float end_dot = end_to_point.dot(plane.normal);
    float abs_origin_dot = fabsf(origin_dot);
    float abs_end_dot = fabsf(end_dot);
    if (abs_origin_dot > eps && abs_end_dot > eps && (origin_dot > 0 ) ^ (end_dot > 0)) {
        float fac = abs_origin_dot / (abs_origin_dot + abs_end_dot);
        intPoint = node.end * fac + node.origin * (1 - fac);
        return true;
    }
    return false;
}

bool ray_int_triangle(Node3D node, Triangle tri, float eps, vec3& intPoint) {
    vec3 e1 = tri.b - tri.a;
    vec3 e2 = tri.c - tri.a;
    vec3 h = node.vector.cross(e2);
    float a = e1.dot(h);

    // if ray is parallel to triangle
    if (fabsf(a) < eps) { return false; }

    float f = 1 / a;
    vec3 s = node.origin - tri.a;
    float u = f * s.dot(h);
    if (u < 0.0f || u > 1.0f) { return false; }
    vec3 q = s.cross(e1);
    float v = f * node.vector.dot(q);
    if (v < 0.0f || u + v > 1.0f) { return false; }

    // now find intersection point
    float t = f * e2.dot(q);
    intPoint = node.origin + node.vector * t; // will save garbage answer to intpoint if t < eps
    return t > eps;
}