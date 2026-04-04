#ifndef MAT3x3_H
#define MAT3x3_H

#include "Vec3.h"
#include <cmath>
#include <cstdio>

class Mat3x3{
private:
    float m[9];

public:

    Mat3x3() : m{1, 0, 0, 0, 1, 0, 0, 0, 1} {}

    Mat3x3(float m0, float m1, float m2, float m3, float m4, float m5, float m6, float m7, float m8)
    : m{m0, m1, m2, m3, m4, m5, m6, m7, m8} {}

    Mat3x3 Transpose(){
        return Mat3x3(
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        );
    }

    float Determinant(){
        return (m[0] * (m[4] * m[8] - m[5]*m[7])) - (m[1] * (m[3] * m[8] - m[5]*m[6])) + (m[2] * (m[3] * m[7] - m[4]*m[6]));
    }

    Mat3x3 Multiply(Mat3x3 other){
        Vec3 col0 = Multiply(Vec3(other.m[0], other.m[3], other.m[6]));
        Vec3 col1 = Multiply(Vec3(other.m[1], other.m[4], other.m[7]));
        Vec3 col2 = Multiply(Vec3(other.m[2], other.m[5], other.m[8]));
        return Mat3x3(
            col0.getX(), col1.getX(), col2.getX(),
            col0.getY(), col1.getY(), col2.getY(),
            col0.getZ(), col1.getZ(), col2.getZ()
        );
    }

    Vec3 Multiply(Vec3 v){
        return Vec3(
            (v.getX() * m[0] + v.getY() * m[1] + v.getZ() * m[2]), //x
            (v.getX() * m[3] + v.getY() * m[4] + v.getZ() * m[5]), //y
            (v.getX() * m[6] + v.getY() * m[7] + v.getZ() * m[8])  //z
        );
    }

    static Mat3x3 Identity(){
        return Mat3x3();
    }

    static Mat3x3 RotateX(float angle){
        float rad = angle * M_PI / 180.0f;
        return Mat3x3(
            1, 0, 0,
            0, cos(rad), -sin(rad),
            0, sin(rad), cos(rad)
        );
    }

    static Mat3x3 RotateY(float angle){
        float rad = angle * M_PI / 180.0f;
        return Mat3x3(
            cos(rad), 0, sin(rad),
            0, 1, 0,
            -sin(rad), 0, cos(rad)
        );
    }

    static Mat3x3 RotateZ(float angle){
        float rad = angle * M_PI / 180.0f;
        return Mat3x3(
            cos(rad), -sin(rad), 0,
            sin(rad), cos(rad), 0,
            0, 0, 1

        );
    }

    void print(){
        printf("%f %f %f\n", m[0], m[1], m[2]);
        printf("%f %f %f\n", m[3], m[4], m[5]);
        printf("%f %f %f\n", m[6], m[7], m[8]);
    }   

};
#endif