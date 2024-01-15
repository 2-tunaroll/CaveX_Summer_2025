#pragma once

#include <cstdio>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

namespace arachnida {
    struct Vector3D {
        float x;
        float y;
        float z;

        Vector3D(float x, float y, float z) : x(x), y(y), z(z) {};

        Vector3D(float x, float y) : x(x), y(y), z(0) {};

        Vector3D() : x(0), y(0), z(0) {};

        inline Vector3D operator + (const Vector3D &v) {
            Vector3D result;
            result.x = x + v.x;
            result.y = y + v.y;
            result.z = z + v.z;
            return result;
        };

        inline Vector3D operator - (const Vector3D &v) {
            Vector3D result;
            result.x = x - v.x;
            result.y = y - v.y;
            result.z = z - v.z;
            return result;
        };

        inline Vector3D operator * (double factor) {
            Vector3D result;
            result.x = x * factor;
            result.y = y * factor;
            result.z = z * factor;
            return result;
        };

        inline Vector3D operator / (double factor) {
            Vector3D result;
            result.x = x / factor;
            result.y = y / factor;
            result.z = z / factor;
            return result;
        };

        inline Vector3D operator = (const Vector3D &v) {
            x = v.x;
            y = v.y;
            z = v.z;
            return *this;
        };

        inline bool operator == (const Vector3D &v) {
            return (x == v.x && y == v.y && z == v.z);
        };

        inline bool operator != (const Vector3D &v) {
            return (x != v.x || y != v.y || z != v.z);
        };

        friend std::ostream& operator << (std::ostream &out, const Vector3D v) {
            out << "x: " << v.x << ", y: " << v.y << ", z: " << v.z;
            return out;
        };
    } Vector3D;


    struct Quaternion {
        float w;
        float x;
        float y;
        float z;

        Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {};

        Quaternion() : w(0), x(0), y(0), z(0) {};

    } Quaternion;
}
