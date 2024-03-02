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
    float roundness;
public:
    toggle_button(Vector2 origin, float width, float height, Color color_untoggled, Color color_toggled, bool reset_value=false, float roundness=0.0){
        this->rec.x = origin.x;
        this->rec.y = origin.y;
        this->rec.width = width;
        this->rec.height = height;
        this->color_toggled = color_toggled;
        this->color_untoggled = color_untoggled;
        this->toggled = reset_value;
        this->roundness = roundness;
    }
    bool process(Vector2 mouse){
        if(CheckCollisionPointRec(mouse, rec)  && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))toggled = !toggled;
        return toggled;
    }
    bool toggle(){
        toggled = !toggled;
        return toggled;
    }
    bool get_value(){
        return toggled;
    }
    void set_value(bool value){
        toggled = value;
    }
    void move(Vector2 origin){
        rec.x = origin.x;
        rec.y = origin.y;
    }
    Rectangle get_rec(){
        return rec;
    }
    void draw(){
        DrawRectangleRounded(rec, roundness, 10, toggled ? color_toggled : color_untoggled);
    }
};

class push_button
{
private:
    Rectangle rec;
    bool pushed;
    bool pushed_last;
    Color color_unpushed;
    Color color_pushed;
    float roundness;
public:
    push_button(Vector2 origin, float width, float height, Color color_unpushed, Color color_pushed, bool reset_value=false, float roundness=0.0){
        this->rec.x = origin.x;
        this->rec.y = origin.y;
        this->rec.width = width;
        this->rec.height = height;
        this->color_pushed = color_pushed;
        this->color_unpushed = color_unpushed;
        this->pushed = reset_value;
        this->roundness = roundness;
    }
    bool process(Vector2 mouse){
        pushed_last = pushed;
        if(CheckCollisionPointRec(mouse, rec)  && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))pushed = 1;
        if(pushed && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) pushed = 0;
        return pushed;
    }
    bool push(){
        pushed = 1;
        return pushed;
    }
    bool unpush(){
        pushed = 0;
        return pushed;
    }
    bool is_pressed(){
        return pushed && !pushed_last;
    }
    bool is_released(){
        return !pushed && pushed_last;
    }
    bool get_value(){
        return pushed;
    }
    void set_value(bool value){
        pushed = value;
    }
    void move(Vector2 origin){
        rec.x = origin.x;
        rec.y = origin.y;
    }
    Rectangle get_rec(){
        return rec;
    }
    void draw(){
        DrawRectangleRounded(rec, roundness, 10, pushed ? color_pushed : color_unpushed);
    }
};