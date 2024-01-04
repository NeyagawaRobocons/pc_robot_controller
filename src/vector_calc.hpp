#pragma once
#include <cmath>
#include "raylib.h"

float dot(Vector2 a, Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

float len_vec2(Vector2 vec) {
    return sqrtf32(vec.x * vec.x + vec.y * vec.y);
}

Vector2 normarize(Vector2 vec) {
    float len = sqrtf32(vec.x * vec.x + vec.y * vec.y);
    if(len == 0.0f) return Vector2{ 0.0f, 0.0f};
    else return Vector2{ vec.x /len, vec.y /len};
}

Vector2 invert_y(Vector2 vec){
    return Vector2{vec.x, -vec.y};
}

Vector2 operator*(Vector2 v, float f){
    return Vector2{v.x*f, v.y*f};
}

Vector2 operator+(Vector2 v1, Vector2 v2){
    return Vector2{v1.x + v2.x, v1.y + v2.y};
}

Vector2 operator-(Vector2 v1, Vector2 v2){
    return Vector2{v1.x - v2.x, v1.y - v2.y};
}