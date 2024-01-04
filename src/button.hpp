#pragma once
#include "raylib.h"
#include "vector_calc.hpp"

class toggle_button
{
private:
    Rectangle rec;
    bool toggled;
    Color color_untoggled;
    Color color_toggled;
public:
    toggle_button(Vector2 origin, float width, float height, Color color_untoggled, Color color_toggled, bool reset_value=false){
        this->rec.x = origin.x;
        this->rec.y = origin.y;
        this->rec.width = width;
        this->rec.height = height;
        this->color_toggled = color_toggled;
        this->color_untoggled = color_untoggled;
        this->toggled = reset_value;
    }
    bool process(Vector2 mouse){
        if(CheckCollisionPointRec(mouse, rec)  && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))toggled = !toggled;
        return toggled;
    }
    void draw(){
        DrawRectangleRec(rec, toggled ? color_toggled : color_untoggled);
    }
};