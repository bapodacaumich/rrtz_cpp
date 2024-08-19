#include "node3d_struct.hpp"
#include "obs.hpp"
#include "triangle_struct.hpp"
#include "utils.hpp"
#include "vec3_struct.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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

bool ray_int_triangle(vec3 origin, vec3 vector, vec3 end, Triangle tri, vec3& intPoint, float eps) {
    // look for any intersections between the ray and triangle (before end-point)
    vec3 e1 = tri.b - tri.a;
    vec3 e2 = tri.c - tri.a;
    vec3 h = vector.cross(e2);
    float a = e1.dot(h);

    // if ray is parallel to triangle
    if (fabsf(a) < eps) { return false; }

    float f = 1 / a;
    vec3 s = origin - tri.a;
    float u = f * s.dot(h);
    if (u < 0.0f || u > 1.0f) { return false; }
    vec3 q = s.cross(e1);
    float v = f * vector.dot(q);
    if (v < 0.0f || u + v > 1.0f) { return false; }

    // now find intersection point
    float t = f * e2.dot(q);
    intPoint = origin + vector * t; // will save garbage answer to intpoint if t <= eps

    // check if intersection point is between start and endpoint
    vec3 origin_end = end - origin;
    vec3 origin_int = intPoint - origin;
    float origin_end_dot = origin_end.dot(origin_int/origin_int.norm());
    float origin_int_norm = origin_int.norm();


    return origin_int_norm > 0 && origin_int_norm < origin_end_dot;
}

bool ray_int_triangle(vec3 origin, vec3 vector, Triangle tri, vec3& intPoint, float eps) {
    // look for any intersections between the ray and triangle (no end point)
    vec3 e1 = tri.b - tri.a;
    vec3 e2 = tri.c - tri.a;
    vec3 h = vector.cross(e2);
    float a = e1.dot(h);

    // if ray is parallel to triangle
    if (fabsf(a) < eps) { return false; }

    float f = 1 / a;
    vec3 s = origin - tri.a;
    float u = f * s.dot(h);
    if (u < 0.0f || u > 1.0f) { return false; }
    vec3 q = s.cross(e1);
    float v = f * vector.dot(q);
    if (v < 0.0f || u + v > 1.0f) { return false; }

    // now find intersection point
    float t = f * e2.dot(q);
    intPoint = origin + vector * t; // will save garbage answer to intpoint if t <= eps
    // std::cout << " inFunction: " << intPoint.x << " " << intPoint.y << " " << intPoint.z << std::endl;
    return t > eps;
}

bool ray_int_triangle(Node3D node, Triangle tri, vec3& intPoint, float eps) {
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
    // std::cout << " inFunction: " << intPoint.x << " " << intPoint.y << " " << intPoint.z << std::endl;
    return t > eps;
}

float heading_change(Node3D node, Node3D next_node) {
    if (node.vector.norm() < 1e-9f || next_node.vector.norm() < 1e-9f) { return 0.0f; }
    float heading_change = acosf(node.vector.dot(next_node.vector) / (node.vector.norm() * next_node.vector.norm() + 1e-9f));
    return heading_change;
}

float heading_change(Node3D node, vec3 vector) {
    return acosf(node.vector.dot(vector) / (node.vector.norm() * vector.norm() + 1e-9f));
}

bool loadCSV(const std::string& filename, std::vector<std::vector<float>>& data, int rowlen){
    /*
    load a csv file into a vector of vectors
    args:
    - filename: std::string, path to csv file
    - data: std::vector<std::vector<float>>, output data
    */
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::vector<float> row;
        std::stringstream ss(line);
        std::string cell;
        int num_cells = 0;
        while (std::getline(ss, cell, ',') && num_cells < rowlen)
        {
            try {
                row.push_back(std::stof(cell)); // Convert string to float and add to row
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid number format: " << cell << std::endl;
                return false;
            } catch (const std::out_of_range& e) {
                std::cerr << "Number out of range: " << cell << std::endl;
                return false;
            }
            ++num_cells;
        }
        data.push_back(row);
    }

    file.close();
    return true;
}

void saveCSV(const std::string& filename, const std::vector<std::vector<float>>& data) {
    /*
    * Save 2d std::vector float to a csv file
    * @param filename: std::string, path to save file
    * @param data: std::vector<std::vector<float>>, data to save
    */


    // Open the file in output mode
    std::ofstream file(filename);
    
    // Check if the file was opened successfully
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    
    // Iterate over the rows
    size_t n_rows = data.size();
    for (size_t i = 0; i < n_rows; ++i) {
        // Iterate over the columns
        size_t n_cols = data[i].size();
        for (size_t j = 0; j < n_cols; ++j) {
            file << std::fixed << std::setprecision(6) << data[i][j]; // Write value
            if (j < n_cols - 1) {
                file << ","; // Separate values with a comma
            }
        }
        file << "\n"; // End of row
    }

    // Close the file
    file.close();
}

void loadCube(std::vector<std::vector<std::vector<float>>>& data, float xs, float xf) {
    /*
    * instantiate cube triangles into data
    * @param data: std::vector<std::vector<std::vector<float>>>, output data
    */
    data = {
        {{xf, 1,-1}, {xf,-1,-1}, {xs, 1,-1}},  {{xf,-1,-1},  {xs, 1,-1},  {xs,-1,-1}}, // base
        {{xs, 1, 1}, {xs,-1, 1}, {xs, 1,-1}},  {{xs,-1, 1},  {xs, 1,-1},  {xs,-1,-1}}, // front
        {{xf, 1,-1}, {xs, 1,-1}, {xs, 1, 1}},  {{xf, 1,-1},  {xs, 1, 1},  {xf, 1, 1}}, // left
        {{xf, 1, 1}, {xf,-1, 1}, {xs, 1, 1}},  {{xf,-1, 1},  {xs, 1, 1},  {xs,-1, 1}}, // top
        {{xf,-1, 1}, {xs,-1, 1}, {xf,-1,-1}},  {{xf,-1,-1},  {xs,-1,-1},  {xs,-1, 1}}, // right
        {{xf, 1,-1}, {xf,-1,-1}, {xf,-1, 1}},  {{xf, 1,-1},  {xf, 1, 1},  {xf,-1, 1}}  // back
    };
}

void convertFlatToTriangle(const std::vector<std::vector<float>>& flatData, std::vector<Triangle>& triangles) {
    /* convert flat 2d object to triangles assuming each triangle is flattened into x1,y1,z1,x2,y2,z2,x3,y3,z3
    */
    for (size_t i = 0; i < flatData.size(); ++i) {
        triangles.push_back(Triangle(
            vec3(flatData[i][0], flatData[i][1], flatData[i][2]),
            vec3(flatData[i][3], flatData[i][4], flatData[i][5]),
            vec3(flatData[i][6], flatData[i][7], flatData[i][8])
        ));
    }
}

void loadStationOBS(std::vector<OBS>& obsVec) {
    /*
    * instantiate station into obstacle objects
    * @param obsVec: std::vector<OBS>, output data
    */
    size_t num_obstacles = 15;
    std::string model_dir = "../data/model_convex/";

    for (size_t i=1; i < num_obstacles+1; ++i) {
        std::string filename = model_dir + std::to_string(i) + ".csv";
        std::vector<std::vector<float>> data;
        loadCSV(filename, data, 3);
        std::vector<Triangle> tris;
        convertFlatToTriangle(data, tris);
        OBS obs = OBS(tris);
        obsVec.push_back(obs);
    }
}

void vecToTri(const std::vector<std::vector<std::vector<float>>>& data, std::vector<Triangle>& tris) {
    /*
    * Convert a vector of vectors of vectors to a vector of triangles
    * @param data: std::vector<std::vector<std::vector<float>>>, input data
    * @param vec: std::vector<Triangles>, output data
    */
    for (size_t i = 0; i < data.size(); ++i) {
        tris.push_back(Triangle(
            vec3(data[i][0][0], data[i][0][1], data[i][0][2]),
            vec3(data[i][1][0], data[i][1][1], data[i][1][2]),
            vec3(data[i][2][0], data[i][2][1], data[i][2][2])
        ));
    }
}