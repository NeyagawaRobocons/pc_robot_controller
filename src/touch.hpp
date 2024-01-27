#pragma once
#include "raylib.h"
#include <vector>

struct touch_element
{
    Vector2 point;
    int id;
    bool is_new;
};


class Touch
{
private:
    std::vector<touch_element> touchs;
public:
    Touch();
    void process(){
        int num = GetTouchPointCount();
        std::vector<touch_element> touchs_new;
        touchs_new.resize(num);
        for (size_t i = 0; i < num; i++)
        {
            touchs_new[i].point = GetTouchPosition(i);
            touchs_new[i].id = GetTouchPointId(i);
            for (auto t : touchs){
                if(t.id == touchs[i].id) touchs_new[i].is_new = true;
            }
        }
        
    }
    const std::vector<touch_element>& operator =(Touch&){
        return touchs;
    }
    const touch_element& operator [](std::size_t i){
        return touchs[i];
    }
};