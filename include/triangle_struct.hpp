#ifndef TRIANGLE_STRUCT_HPP
#define TRIANGLE_STRUCT_HPP

#include "vec3_struct.hpp"

struct Triangle {
    vec3 a, b, c;
    Triangle () {
        a = vec3();
        b = vec3();
        c = vec3();
    }

    Triangle (vec3 a, vec3 b, vec3 c) {
        this->a = a;
        this->b = b;
        this->c = c;
    }
};

#endif // TRIANGLE_STRUCT_HPP