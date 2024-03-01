#pragma once
#include <raylib.h>
#include <raymath.h>
#include <geometry_msgs/msg/pose.hpp>
#include "cclp/include/lines_and_points.hpp"

class PoseAdjust {
private:
    Vector2 mouse_map_begin = {0, 0};
    Vector2 mouse_map_end = {0, 0};
    Vector2 map_draw_origin;
    float map_scale;
    geometry_msgs::msg::Pose pose;
public:
    PoseAdjust(Vector2 map_draw_origin, float map_scale){
        this->map_draw_origin = map_draw_origin;
        this->map_scale = map_scale;
    }
    bool process(Vector2 mouse) {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector2 mouse_map = Vector2Subtract(mouse, map_draw_origin);
            mouse_map = Vector2Divide(mouse_map, {map_scale, -map_scale});
            mouse_map_begin = mouse_map;
        }
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
                Vector2 mouse_map = Vector2Subtract(mouse, map_draw_origin);
                mouse_map = Vector2Divide(mouse_map, {map_scale, -map_scale});
                mouse_map_end = mouse_map;
        }
        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            pose.position.x = mouse_map_begin.x;
            pose.position.y = mouse_map_begin.y;
            pose.position.z = 0;
            Quaternion q = QuaternionFromEuler(0, 0, std::atan2(mouse_map_end.y - mouse_map_begin.y, mouse_map_end.x - mouse_map_begin.x));
            pose.orientation.x = q.x;
            pose.orientation.y = q.y;
            pose.orientation.z = q.z;
            pose.orientation.w = q.w;
            mouse_map_begin = {0, 0};
            mouse_map_end = {0, 0};
            return true;
        }
        return false;
    }

    void draw() {
        if(IsMouseButtonDown(MOUSE_LEFT_BUTTON)){
            draw_line_scale_y_inv({mouse_map_begin, mouse_map_end}, map_scale, map_draw_origin, RED);
        }
    }

    geometry_msgs::msg::Pose get_pose() {
        return pose;
    }

    void set_map_draw_origin(Vector2 map_draw_origin) {
        this->map_draw_origin = map_draw_origin;
    }

    void set_map_scale(float map_scale) {
        this->map_scale = map_scale;
    }
        
};
