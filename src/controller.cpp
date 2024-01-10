#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mecha_control/msg/actuator_commands.hpp"
#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <cmath>
#include "raylib.h"
#include "vector_calc.hpp"
#include "slider.hpp"
#include "button.hpp"


class Controller : public rclcpp::Node {
public:
    Controller() : Node("controller") {
        omni_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("input_vel", 10);
        daiza_publisher_ = this->create_publisher<mecha_control::msg::ActuatorCommands>("daiza_clamp", 10);
        hina_publisher_ = this->create_publisher<mecha_control::msg::ActuatorCommands>("hina_dastpan", 10);
        ui_thread_ = std::thread(&Controller::ui_main, this);
    }
private:
    void ui_main(){
        // Initialization
        //--------------------------------------------------------------------------------------
        const int screenWidth = 1280;
        const int screenHeight = 720;

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

        bool drag_vec = 0;

        slider sl({1000, 400}, {1100, 400}, 5.0f, BLUE);
        toggle_button tb({1000, 300}, 100, 50, GRAY, BLUE);

        toggle_button daiza_cyl12({600, 600}, 50, 30, GRAY, PINK);
        toggle_button daiza_cyl3({670, 600}, 50, 30, GRAY, PINK);
        toggle_button daiza_cyl4({740, 600}, 50, 30, GRAY, PINK);

        int count = 0;

        SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
        //--------------------------------------------------------------------------------------

        // Main game loop
        while (!WindowShouldClose())    // Detect window close button or ESC key
        {
            // Update
            //----------------------------------------------------------------------------------
            Vector2 mouse = GetMousePosition();
            sl.process(mouse);
            tb.process(mouse);
            daiza_cyl12.process(mouse);
            daiza_cyl3.process(mouse);
            daiza_cyl4.process(mouse);

            if(IsKeyPressed(KEY_ONE)){
                daiza_cyl12.toggle();
            }
            if(IsKeyPressed(KEY_TWO)){
                daiza_cyl3.toggle();
            }
            if(IsKeyPressed(KEY_THREE)){
                daiza_cyl4.toggle();
            }
            // calculate robot vec from key
            if(drag_vec){
                robot.v_rec = invert_y(mouse - robot.origin) * (1.0f / scale);
                std::cout << "dragging vector" << std::endl;
            }else{
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
                robot.v_rec = normarize(robot.v_rec)*1.0;
            }
            if (IsKeyDown(KEY_Q)){
                robot.rot = 1.0;
            }else if (IsKeyDown(KEY_E)){
                robot.rot = -1.0;
            }else{
                robot.rot = 0;
            }
            for (size_t i = 0; i < 3; i++)
            {
                robot.v_tire[i] = (dot(robot.e_tire[i], robot.v_rec) + robot.rot * robot.l) / robot.r_tire;
            }
            //publish motor
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {robot.v_tire[0], robot.v_tire[1], robot.v_tire[2]};
            for (size_t i = 0; i < 3; i++)
            {
                robot.v_tire_draw[i].x = robot.e_tire[i].x * robot.v_tire[i] * scale_vec + robot.r_draw[i].x;
                robot.v_tire_draw[i].y = - robot.e_tire[i].y * robot.v_tire[i] * scale_vec + robot.r_draw[i].y;
            }
            //publish daiza
            auto daiza_message = mecha_control::msg::ActuatorCommands();
            daiza_message.cylinder_states = {daiza_cyl12.get_value(), daiza_cyl12.get_value(), daiza_cyl3.get_value(), daiza_cyl4.get_value()};
            count++;
            if(count >= 3){
                omni_publisher_->publish(message);
                daiza_publisher_->publish(daiza_message);
                count = 0;
            }
            // Draw
            //----------------------------------------------------------------------------------
            BeginDrawing();

                ClearBackground(RAYWHITE);

                sl.draw();
                tb.draw();
                daiza_cyl12.draw();
                daiza_cyl3.draw();
                daiza_cyl4.draw();

                DrawSplineLinear(robot.r_draw, 4, 4.0f, GRAY);
                DrawSplineSegmentLinear(robot.origin, invert_y(robot.v_rec*scale)+robot.origin, 4.0f, PINK);
                DrawCircleV(invert_y(robot.v_rec*scale)+robot.origin, 8.0f, PINK);
                for (size_t i = 0; i < 3; i++)
                {
                    DrawSplineSegmentLinear(robot.r_draw[i], robot.v_tire_draw[i], 4.0f, BLUE);
                }
                
                DrawText("omni! 3 wheel!", 400, 20, 20, LIGHTGRAY);

                std::ostringstream ss;
                ss << std::fixed;
                ss << "v(" << robot.v_rec.x << ", " << robot.v_rec.y << ")" << std::endl << std::endl;
                for (size_t i = 0; i < 3; i++)
                {
                    ss << "omega[" << i << "] : " << robot.v_tire[i] << std::endl;
                }
                DrawText(ss.str().c_str(), 1000, 200, 20, GRAY);

            EndDrawing();
            //----------------------------------------------------------------------------------
        }

        // De-Initialization
        //--------------------------------------------------------------------------------------
        CloseWindow();        // Close window and OpenGL context
        //--------------------------------------------------------------------------------------

        return ;
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omni_publisher_;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr daiza_publisher_;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr hina_publisher_;
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