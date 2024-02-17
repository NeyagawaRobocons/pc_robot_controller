#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mecha_control/msg/actuator_commands.hpp"
#include "mecha_control/msg/mecha_state.hpp"
#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <cmath>
#include "raylib.h"
#include "vector_calc.hpp"
#include "slider.hpp"
#include "button.hpp"
#include "label.hpp"


class Controller : public rclcpp::Node {
public:
    Controller() : Node("controller") {
        omni_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("input_vel", 10);
        daiza_publisher_ = this->create_publisher<mecha_control::msg::ActuatorCommands>("daiza_clamp", 10);
        hina_publisher_ = this->create_publisher<mecha_control::msg::ActuatorCommands>("hina_dastpan", 10);
        mecha_publisher_ = this->create_publisher<mecha_control::msg::MechaState>("mecha_state", 10);
        hina_servo_publisher_ = this->create_publisher<std_msgs::msg::String>("direct_mecha_command", 10);
        ui_thread_ = std::thread(&Controller::ui_main, this);
    }
private:
    void ui_main(){
        // Initialization
        //--------------------------------------------------------------------------------------
        int screenWidth = 1280;
        int screenHeight = 720;

        SetConfigFlags(FLAG_WINDOW_RESIZABLE);
        InitWindow(screenWidth, screenHeight, "omni controller");
        screenWidth = GetScreenWidth();
        screenHeight = GetScreenHeight();

        float scale = 300;
        float scale_vec = 0.051 * 300;

        struct{
            Vector2 origin; // robot origin
            Vector2 v_rec; // robot rectangular vector
            float rot; // robot rotate rate
            float l;   // r1,2,3 len
            float r_tire;   // tire radius
            Vector2 r[4]; // vector to tire from origin
            Vector2 r_draw[5]; // vector to tire from origin
            Vector2 e_tire[4]; // vector of tire direction normarized
            float v_tire[4]; // vector of tire
            Vector2 v_tire_draw[4]; // vector of tire direction normarized
        } robot;
        robot.origin = Vector2{screenWidth/2, screenHeight/2};
        robot.l = 0.3;
        robot.r_tire = 0.051;
        robot.r[0] = Vector2{robot.l * sqrtf32(2) * 0.5f, robot.l * sqrtf32(2) * 0.5f};
        robot.r[1] = Vector2{- robot.l * sqrtf32(2) * 0.5f, robot.l * sqrtf32(2) * 0.5f};
        robot.r[2] = Vector2{- robot.l * sqrtf32(2) * 0.5f, - robot.l * sqrtf32(2) * 0.5f};
        robot.r[3] = Vector2{robot.l * sqrtf32(2) * 0.5f, - robot.l * sqrtf32(2) * 0.5f};
        robot.e_tire[0] = Vector2{- sqrtf32(2) * 0.5f, sqrtf32(2) * 0.5f};
        robot.e_tire[1] = Vector2{- sqrtf32(2) * 0.5f, - sqrtf32(2) * 0.5f};
        robot.e_tire[2] = Vector2{sqrtf32(2) * 0.5f, - sqrtf32(2) * 0.5f};
        robot.e_tire[3] = Vector2{sqrtf32(2) * 0.5f, sqrtf32(2) * 0.5f};
        for (size_t i = 0; i < 4; i++)
        {
            robot.r_draw[i].x = scale * robot.r[i].x + robot.origin.x;
            robot.r_draw[i].y = - scale * robot.r[i].y + robot.origin.y;
        }
        robot.r_draw[4] = robot.r_draw[0];

        bool drag_vec = 0;

        // slider sl({1000, 400}, {1100, 400}, 5.0f, BLUE);
        // toggle_button tb({1000, 300}, 100, 50, GRAY, BLUE);

        label title_daize("daiza_clamp mech", {screenWidth -210, 370}, 20);
        toggle_button daiza_cyl12({screenWidth -200, 400}, 130, 70, GRAY, PINK);
        toggle_button daiza_cyl3({screenWidth -200, 500}, 130, 70, GRAY, PINK);
        toggle_button daiza_cyl4({screenWidth -200, 600}, 130, 70, GRAY, PINK);
        label label_cyl12("cylinder 1,2", {screenWidth -190, 425}, 20);
        label label_cyl3("cylinder 3", {screenWidth -190, 525}, 20);
        label label_cyl4("cylinder 4", {screenWidth -190, 625}, 20);

        label title_hina("hina_dustpan mech", {screenWidth -210, 70}, 20);
        toggle_button hina_expand({screenWidth -200, 100}, 130, 70, GRAY, PINK);
        slider hina_angle({screenWidth -200, 220}, {screenWidth -70, 220}, 5.0f, BLUE, 0.5);
        slider servo1({screenWidth -200, 280}, {screenWidth -70, 280}, 5.0f, BLUE, 0.5);
        slider servo2({screenWidth -200, 340}, {screenWidth -70, 340}, 5.0f, BLUE, 0.5);
        label label_hina_expand("hina expand", {screenWidth -190, 125}, 20);
        label label_hina_angle("hina angle", {screenWidth -190, 190}, 20);
        label label_servo1("servo1", {screenWidth -190, 250}, 20);
        label label_servo2("servo2", {screenWidth -190, 310}, 20);

        // label title_mech("mech state", {screenWidth -210, 70}, 20);
        // push_button daiza_seq({screenWidth -250, 100}, 180, 70, GRAY, PINK);
        // push_button hina_seq({screenWidth -250, 200}, 180, 70, GRAY, PINK);
        // toggle_button hina_servo({screenWidth -250, 300}, 180, 70, GRAY, PINK);
        // toggle_button bonbori_seq({screenWidth -200, 500}, 130, 70, GRAY, PINK);
        // uint8_t daiza_state_value = 4;
        // uint8_t hina_state_value = 4;
        // uint8_t daiza_state_value_prev = 4;
        // uint8_t hina_state_value_prev = 4;
        // bool hina_servo_prev = 0;
        // bool bonbori_prev = 0;
        // char text_label_daiza_seq[14];
        // char text_label_hina_seq[14];
        // sprintf(text_label_daiza_seq, "daiza seq : %d", daiza_state_value);
        // sprintf(text_label_hina_seq, "hina  seq : %d", hina_state_value);
        // label label_daiza_seq(text_label_daiza_seq, {screenWidth -240, 125}, 20);
        // label label_hina_seq(text_label_hina_seq, {screenWidth -240, 225}, 20);
        // label label_hina_servo("hina servo", {screenWidth -240, 325}, 20);
        // label label_bonbori_seq("bonbori", {screenWidth -190, 525}, 20);

        int count = 0;

        SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
        //--------------------------------------------------------------------------------------

        // Main game loop
        while (!WindowShouldClose())    // Detect window close button or ESC key
        {
            // Update
            //----------------------------------------------------------------------------------
            //resize action
            screenWidth = GetScreenWidth();
            screenHeight = GetScreenHeight();
            title_daize.move({screenWidth -210, 370});
            daiza_cyl12.move({screenWidth -200, 400});
            daiza_cyl3.move({screenWidth -200, 500});
            daiza_cyl4.move({screenWidth -200, 600});
            label_cyl12.move({screenWidth -190, 425});
            label_cyl3.move({screenWidth -190, 525});
            label_cyl4.move({screenWidth -190, 625});
            title_hina.move({screenWidth -210, 70});
            hina_expand.move({screenWidth -200, 100});
            hina_angle.move({screenWidth -200, 220});
            servo1.move({screenWidth -200, 280});
            servo2.move({screenWidth -200, 340});
            label_hina_expand.move({screenWidth -190, 125});
            label_hina_angle.move({screenWidth -190, 190});
            label_servo1.move({screenWidth -190, 250});
            label_servo2.move({screenWidth -190, 310});
            // title_mech.move({screenWidth -210, 70});
            // daiza_seq.move({screenWidth -250, 100});
            // hina_seq.move({screenWidth -250, 200});
            // hina_servo.move({screenWidth -250, 300});
            // bonbori_seq.move({screenWidth -200, 500});
            // label_daiza_seq.move({screenWidth -240, 125});
            // label_hina_seq.move({screenWidth -240, 225});
            // label_hina_servo.move({screenWidth -240, 325});
            // label_bonbori_seq.move({screenWidth -190, 525});
            robot.origin = Vector2{screenWidth/2, screenHeight/2};
            for (size_t i = 0; i < 4; i++)
            {
                robot.r_draw[i].x = scale * robot.r[i].x + robot.origin.x;
                robot.r_draw[i].y = - scale * robot.r[i].y + robot.origin.y;
            }
            robot.r_draw[4] = robot.r_draw[0];
            
            // logics
            //

            Vector2 mouse = GetMousePosition();
            // sl.process(mouse);
            // tb.process(mouse);
            daiza_cyl12.process(mouse);
            daiza_cyl3.process(mouse);
            daiza_cyl4.process(mouse);
            hina_expand.process(mouse);
            hina_angle.process(mouse);
            servo1.process(mouse);
            servo2.process(mouse);
            // daiza_seq.process(mouse);
            // hina_seq.process(mouse);
            // hina_servo.process(mouse);
            // bonbori_seq.process(mouse);

            // if(IsKeyPressed(KEY_ONE)){
            //     daiza_seq.push();
            // }else if(IsKeyReleased(KEY_ONE)){
            //     daiza_seq.unpush();
            // }
            // if(IsKeyPressed(KEY_TWO)){
            //     hina_seq.push();
            // }else if(IsKeyReleased(KEY_TWO)){
            //     hina_seq.unpush();
            // }
            // if(IsKeyPressed(KEY_THREE)){
            //     hina_servo.toggle();
            // }
            // if(IsKeyPressed(KEY_FOUR)){
            //     bonbori_seq.toggle();
            // }
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
                robot.v_rec = normarize(robot.v_rec)*2.0;
            }
            if (IsKeyDown(KEY_Q)){
                robot.rot = 4.0;
            }else if (IsKeyDown(KEY_E)){
                robot.rot = -4.0;
            }else{
                robot.rot = 0;
            }
            for (size_t i = 0; i < 4; i++)
            {
                robot.v_tire[i] = (dot(robot.e_tire[i], robot.v_rec) + robot.rot * robot.l) / robot.r_tire;
            }
            //publish motor
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {robot.v_tire[0], robot.v_tire[1], robot.v_tire[2], robot.v_tire[3]};
            for (size_t i = 0; i < 4; i++)
            {
                robot.v_tire_draw[i].x = robot.e_tire[i].x * robot.v_tire[i] * scale_vec + robot.r_draw[i].x;
                robot.v_tire_draw[i].y = - robot.e_tire[i].y * robot.v_tire[i] * scale_vec + robot.r_draw[i].y;
            }
            //publish daiza
            auto daiza_message = mecha_control::msg::ActuatorCommands();
            daiza_message.cylinder_states = {daiza_cyl12.get_value(), daiza_cyl12.get_value(), daiza_cyl3.get_value(), daiza_cyl4.get_value()};
            // publish hina
            auto hina_message = mecha_control::msg::ActuatorCommands();
            hina_message.cylinder_states = {0, 0};
            hina_message.motor_expand = {hina_expand.get_value()};
            std::cout << "hina_expand : " << hina_expand.get_value() << std::endl;
            hina_message.motor_positions = {(hina_angle.get_value() -0.5)*PI, (servo1.get_value() -0.5)*PI, (servo2.get_value() - 0.5)*PI};
            // // publish mech state
            // if(daiza_seq.is_pressed()){
            //     daiza_state_value += 1;
            //     if (daiza_state_value >= 5) daiza_state_value = 1;
            // }
            // if(hina_seq.is_pressed()){
            //     hina_state_value += 1;
            //     if (hina_state_value >= 5) hina_state_value = 1;
            // }
            // sprintf(text_label_daiza_seq, "daiza seq : %d", daiza_state_value);
            // sprintf(text_label_hina_seq, "hina  seq : %d", hina_state_value);
            // label_daiza_seq.set_text(text_label_daiza_seq);
            // label_hina_seq.set_text(text_label_hina_seq);
            // auto mech_message = mecha_control::msg::MechaState();
            // mech_message.daiza_state = daiza_state_value != daiza_state_value_prev ? daiza_state_value : 0;
            // mech_message.hina_state = hina_state_value != hina_state_value_prev ? hina_state_value : 0;
            // mech_message.bonbori_state = bonbori_seq.get_value() == 1 && bonbori_prev == 0;
            // // publis hina servo direct mecha command
            // auto hina_servo_msg = std_msgs::msg::String();
            // if(bonbori_seq.get_value() == 0 && bonbori_prev == 1) hina_servo_msg.data = "bonboriOff";
            // if(hina_servo.get_value()){
            //     hina_servo_msg.data = "90degree";
            // }else{
            //     hina_servo_msg.data = "0degree";
            // }
            count++;
            if(count >= 3){
                omni_publisher_->publish(message);
                daiza_publisher_->publish(daiza_message);
                hina_publisher_->publish(hina_message);
                // mecha_publisher_->publish(mech_message);
                // daiza_state_value_prev = daiza_state_value;
                // hina_state_value_prev = hina_state_value;
                // bonbori_prev = bonbori_seq.get_value();
                // if(hina_servo.get_value() != hina_servo_prev) hina_servo_publisher_->publish(hina_servo_msg);
                // hina_servo_prev = hina_servo.get_value();
                count = 0;
            }
            // Draw
            //----------------------------------------------------------------------------------
            BeginDrawing();

                ClearBackground(RAYWHITE);

                // sl.draw();
                // tb.draw();
                title_daize.draw();
                daiza_cyl12.draw();
                daiza_cyl3.draw();
                daiza_cyl4.draw();
                label_cyl12.draw();
                label_cyl3.draw();
                label_cyl4.draw();
                title_hina.draw();
                hina_expand.draw();
                hina_angle.draw();
                servo1.draw();
                servo2.draw();
                label_hina_expand.draw();
                label_hina_angle.draw();
                label_servo1.draw();
                label_servo2.draw();
                // title_mech.draw();
                // daiza_seq.draw();
                // hina_seq.draw();
                // hina_servo.draw();
                // bonbori_seq.draw();
                // label_daiza_seq.draw();
                // label_hina_seq.draw();
                // label_hina_servo.draw();
                // label_bonbori_seq.draw();

                DrawSplineLinear(robot.r_draw, 5, 4.0f, GRAY);
                DrawSplineSegmentLinear(robot.origin, invert_y(robot.v_rec*scale)+robot.origin, 4.0f, PINK);
                DrawCircleV(invert_y(robot.v_rec*scale)+robot.origin, 8.0f, PINK);
                for (size_t i = 0; i < 4; i++)
                {
                    DrawSplineSegmentLinear(robot.r_draw[i], robot.v_tire_draw[i], 4.0f, BLUE);
                }

                std::ostringstream ss;
                ss << std::fixed;
                ss << "v(" << robot.v_rec.x << ", " << robot.v_rec.y << ")" << std::endl << std::endl;
                for (size_t i = 0; i < 4; i++)
                {
                    ss << "omega[" << i << "] : " << robot.v_tire[i] << std::endl;
                }
                DrawText(ss.str().c_str(), 40, 200, 20, GRAY);

            EndDrawing();
            //----------------------------------------------------------------------------------
        }

        // De-Initialization
        //--------------------------------------------------------------------------------------
        CloseWindow();        // Close window and OpenGL context
        //--------------------------------------------------------------------------------------
        rclcpp::shutdown();
        return ;
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omni_publisher_;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr daiza_publisher_;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr hina_publisher_;
    rclcpp::Publisher<mecha_control::msg::MechaState>::SharedPtr mecha_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hina_servo_publisher_;
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