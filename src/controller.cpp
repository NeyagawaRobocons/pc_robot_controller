#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <cmath>
#include "raylib.h"

float dot(Vector2 a, Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

Vector2 normarize(Vector2 vec) {
    float len = sqrtf32(vec.x * vec.x + vec.y * vec.y);
    if(len == 0.0f) return Vector2{ 0.0f, 0.0f};
    else return Vector2{ vec.x /len, vec.y /len};
}

Vector2 operator*(Vector2 v, float f){
    return Vector2{v.x*f, v.y*f};
}

Vector2 operator+(Vector2 v1, Vector2 v2){
    return Vector2{v1.x + v2.x, v1.y + v2.y};
}

class Controller : public rclcpp::Node {
public:
    Controller() : Node("controller") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor_3omini", 10);
        ui_thread_ = std::thread(&Controller::ui_main, this);
    }
private:
    void ui_main(){
        // Initialization
        //--------------------------------------------------------------------------------------
        const int screenWidth = 800;
        const int screenHeight = 450;

        InitWindow(screenWidth, screenHeight, "omni controller");

        float scale = 300;
        float scale_vec = 0.051 * 300;

        struct{
            Vector2 origin; // robot origin
            Vector2 v_rec; // robot rectangular vector
            float rot; // robot rotate rate
            float l;   // r1,2,3 len
            float r_tire;   // tire radius
            Vector2 r[3]; // vector to tire from origin
            Vector2 r_draw[4]; // vector to tire from origin
            Vector2 e_tire[3]; // vector of tire direction normarized
            float v_tire[3]; // vector of tire
            Vector2 v_tire_draw[3]; // vector of tire direction normarized
        } robot;
        robot.origin = Vector2{screenWidth/2, screenHeight/2};
        robot.l = 0.3;
        robot.r_tire = 0.051;
        robot.r[0] = Vector2{robot.l * sqrtf32(3) * 0.5f, robot.l * 0.5f};
        robot.r[1] = Vector2{- robot.l * sqrtf32(3) * 0.5f, robot.l * 0.5f};
        robot.r[2] = Vector2{0, - robot.l};
        robot.e_tire[0] = Vector2{- 0.5f, sqrtf32(3) * 0.5f};
        robot.e_tire[1] = Vector2{- 0.5f, - sqrtf32(3) * 0.5f};
        robot.e_tire[2] = Vector2{1.0f, 0.0f};
        for (size_t i = 0; i < 3; i++)
        {
            robot.r_draw[i].x = scale * robot.r[i].x + robot.origin.x;
            robot.r_draw[i].y = - scale * robot.r[i].y + robot.origin.y;
        }
        robot.r_draw[3] = robot.r_draw[0];
        

        Rectangle button0{0};
        button0.x = screenWidth - 150;
        button0.y = 100;
        button0.width = 100;
        button0.height = 30;
        bool button0_down = false;

        SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
        //--------------------------------------------------------------------------------------

        // Main game loop
        while (!WindowShouldClose())    // Detect window close button or ESC key
        {
            // Update
            //----------------------------------------------------------------------------------
            Vector2 mouse = GetMousePosition();
            if(CheckCollisionPointRec(mouse, button0) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) button0_down = true;
            if(button0_down) if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) button0_down = false;

            // calculate robot vec from key
            if (IsKeyDown(KEY_W)){
                robot.v_rec.y = 1;
            }else if (IsKeyDown(KEY_S)){
                robot.v_rec.y = -1;
            }else{
                robot.v_rec.y = 0;
            }
            if (IsKeyDown(KEY_D)){
                robot.v_rec.x = 1;
            }else if (IsKeyDown(KEY_A)){
                robot.v_rec.x = -1;
            }else{
                robot.v_rec.x = 0;
            }
            if (IsKeyDown(KEY_Q)){
                robot.rot = 2.0;
            }else if (IsKeyDown(KEY_E)){
                robot.rot = -2.0;
            }else{
                robot.rot = 0;
            }
            robot.v_rec = normarize(robot.v_rec)*1.0;
            for (size_t i = 0; i < 3; i++)
            {
                robot.v_tire[i] = (dot(robot.e_tire[i], robot.v_rec) + robot.rot * robot.l) / robot.r_tire;
            }
            //publish motor
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {robot.v_tire[0], robot.v_tire[1], robot.v_tire[2]};
            publisher_->publish(message);
            for (size_t i = 0; i < 3; i++)
            {
                robot.v_tire_draw[i].x = robot.e_tire[i].x * robot.v_tire[i] * scale_vec + robot.r_draw[i].x;
                robot.v_tire_draw[i].y = - robot.e_tire[i].y * robot.v_tire[i] * scale_vec + robot.r_draw[i].y;
            }
            

            // Draw
            //----------------------------------------------------------------------------------
            BeginDrawing();

                ClearBackground(RAYWHITE);

                DrawSplineLinear(robot.r_draw, 4, 4.0f, GRAY);
                DrawSplineSegmentLinear(robot.origin, Vector2{robot.v_rec.x*scale+robot.origin.x, -robot.v_rec.y*scale+robot.origin.y}, 4.0f, PINK);
                for (size_t i = 0; i < 3; i++)
                {
                    DrawSplineSegmentLinear(robot.r_draw[i], robot.v_tire_draw[i], 4.0f, BLUE);
                }
                
                DrawText("omni! 3 wheel!", 336, 200, 20, LIGHTGRAY);

                DrawRectangleRec(button0, button0_down ? DARKBLUE : SKYBLUE);

                std::ostringstream ss;
                ss << std::fixed;
                ss << "v(" << robot.v_rec.x << ", " << robot.v_rec.y << ")" << std::endl << std::endl;
                for (size_t i = 0; i < 3; i++)
                {
                    ss << "omega[" << i << "] : " << robot.v_tire[i] << std::endl;
                }
                DrawText(ss.str().c_str(), 550, 200, 20, GRAY);

            EndDrawing();
            //----------------------------------------------------------------------------------
        }

        // De-Initialization
        //--------------------------------------------------------------------------------------
        CloseWindow();        // Close window and OpenGL context
        //--------------------------------------------------------------------------------------

        return ;
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread ui_thread_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}