#pragma once
#include "raylib.h"
#include "vector_calc.hpp"

class slider
{
private:
    float width;
    float len;
    Vector2 start;
    Vector2 end;
    float value;
    bool dragging;
    Color color;
public:
    slider(Vector2 start, Vector2 end, float width, Color color, float reset_value=0.0){
        this->start = start;
        this->end = end;
        this->width = width;
        this->len = len_vec2(end - start);
        this->color = color;
        this->dragging = false;
        this->value = reset_value;
    }
    float process(Vector2 mouse){
        if(CheckCollisionPointCircle(mouse, start + (end - start) * value, width*4)  && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) dragging = true;
        if(dragging && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) dragging = false;

        if(dragging){
            value = dot(mouse - start, normarize(end - start)) / len;
            if (value < 0)value = 0;
            if (value > 1)value = 1;
        }
        return value;
    }
    float get_value(){
        return value;
    }
    void draw(){
        DrawSplineSegmentLinear(start, end, width, GRAY);
        DrawSplineSegmentLinear(start, start + (end - start) * value, width, color);
        DrawCircleV(start + (end - start) * value, width*2, color);
    }
};
