#pragma once
#include "raylib.h"
#include "vector_calc.hpp"

class label
{
private:
    Vector2 origin;
    const char *text;
    int fontSize;
    Color color;
public:
    label(const char *text, Vector2 origin, int fontSize, Color color=BLACK){
        this->text = text;
        this->origin = origin;
        this->fontSize = fontSize;
        this->color = color;
    }
    void set_text(const char *text){
        this->text = text;
    }
    void move(Vector2 origin){
        this->origin = origin;
    }
    void draw(){
        DrawText(text, origin.x, origin.y, fontSize, color);
    }
};
