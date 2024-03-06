#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <cmath>
#include "raylib.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nucleo_agent/msg/actuator_commands.hpp"
#include "cclp/msg/line_array.hpp"
#include "cclp/msg/map_request.hpp"
#include "pure_pursuit/msg/path2_d_with_speed.hpp"
#include "mecha_control/msg/mech_action.hpp"
#include "pure_pursuit/action/path_and_feedback.hpp"
#include "pure_pursuit/srv/get_path.hpp"
#include "mecha_control/action/daiza_cmd.hpp"
#include "mecha_control/action/hina_cmd.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vector_calc.hpp"
#include "slider.hpp"
#include "button.hpp"
#include "label.hpp"
#include "touch.hpp"
#include "robot.hpp"
#include "pose_adjust.hpp"
#include "robot_action.hpp"
#include "robot_action_list.hpp"
#include "cclp/include/lines_and_points.hpp"
#include "../include/robot_action.hpp"


class AutoController : public rclcpp::Node {
public:
    AutoController() : Node("auto_controller"), robot_actions_manager_(path_and_feedback_client_, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_) {
        std::string input_vel_topic = this->declare_parameter("input_vel_topic", "input_vel");
        std::string robot_vel_topic = this->declare_parameter("robot_vel_topic", "robot_vel");
        std::string path_topic = this->declare_parameter("path_topic", "robot_path");
        std::string laser1_scan = this->declare_parameter("laser1_scan_topic", "scan1");
        std::string laser2_scan = this->declare_parameter("laser2_scan_topic", "scan2");
        std::string line_map = this->declare_parameter("line_map_topic", "line_map");
        std::string map_request = this->declare_parameter("map_request_topic", "map_request");
        std::string mouse_point_pub = this->declare_parameter("mouse_point_pub_topic", "initial_pose");
        std::string switchs_topic = this->declare_parameter("switchs_topic", "control_switches");
        map_frame_ = this->declare_parameter("map_frame", "map");
        base_frame_ = this->declare_parameter("base_frame", "corrected_base_link");

        // Create topic subscriptions and publishers
        _input_vel_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            input_vel_topic, 10, std::bind(&AutoController::input_vel_callback, this, std::placeholders::_1));
        _robot_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            robot_vel_topic, 10, std::bind(&AutoController::robot_vel_callback, this, std::placeholders::_1));
        _path_sub = this->create_subscription<nav_msgs::msg::Path>(
            path_topic, 10, std::bind(&AutoController::path_callback, this, std::placeholders::_1));
        laser1_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser1_scan, 10, std::bind(&AutoController::Laser1ScanCallback, this, std::placeholders::_1));
        laser2_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser2_scan, 10, std::bind(&AutoController::Laser2ScanCallback, this, std::placeholders::_1));
        line_map_sub_ = this->create_subscription<cclp::msg::LineArray>(
            line_map, 10, std::bind(&AutoController::line_map_callback, this, std::placeholders::_1));
        map_request_pub_ = this->create_publisher<cclp::msg::MapRequest>(map_request, 10);
        mouse_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(mouse_point_pub, 10);
        get_path_client_ = this->create_client<pure_pursuit::srv::GetPath>("get_path");
        path_and_feedback_client_ = rclcpp_action::create_client<pure_pursuit::action::PathAndFeedback>(this, "path_and_feedback");
        daiza_cmd_client_ = rclcpp_action::create_client<mecha_control::action::DaizaCmd>(this, "daiza_cmd");
        hina_cmd_client_ = rclcpp_action::create_client<mecha_control::action::HinaCmd>(this, "hina_cmd");
        // bonbori_srv_client_ = this->create_client<std_srvs::srv::SetBool>("set_bonbori");
        bonbori_msg_pub_ = this->create_publisher<std_msgs::msg::Bool>("bonbori_msg", 10);
        switchs_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            switchs_topic, 10, std::bind(&AutoController::switchs_callback, this, std::placeholders::_1));
        // Create a tf buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        

        ui_thread_ = std::thread(&AutoController::ui_main, this);
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _input_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _robot_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser1_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser2_scan_sub_;
    rclcpp::Subscription<cclp::msg::LineArray>::SharedPtr line_map_sub_;
    rclcpp::Publisher<cclp::msg::MapRequest>::SharedPtr map_request_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mouse_point_pub_;
    rclcpp::Client<pure_pursuit::srv::GetPath>::SharedPtr get_path_client_;
    rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr path_and_feedback_client_;
    rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_client_;
    rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_client_;
    // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bonbori_msg_pub_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr switchs_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string base_frame_;
    std::mutex points1_mutex;
    std::vector<Vector2> points1;
    std::mutex points2_mutex;
    std::vector<Vector2> points2;
    std::mutex lines_mutex;
    std::vector<Line> lines;
    std::thread ui_thread_;
    Vector3 robot_vec_;
    float input_vel_[4];
    std::vector<Line> path_;
    std::array<bool, 4> switchs_;
    RobotActionsManager robot_actions_manager_;


    // lidar1 parameters
    double lidar1_offset_x_ = - 0.2699;
    double lidar1_offset_y_ = 0.2699;
    double lidar1_offset_theta_ = -0.78539816339745;
    bool lidar1_invert_x_ = true;
    double lidar1_circle_mask_radius_ = 0.5;
    double lidar1_circle_mask_center_x_ = 0.0;
    double lidar1_circle_mask_center_y_ = 0.0;
    // lidar2 parameters
    double lidar2_offset_x_ = 0.2699;
    double lidar2_offset_y_ = - 0.2699;
    double lidar2_offset_theta_ = 2.3561944901923;
    bool lidar2_invert_x_ = true;
    double lidar2_circle_mask_radius_ = 0.5;
    double lidar2_circle_mask_center_x_ = 0.0;
    double lidar2_circle_mask_center_y_ = 0.0;


    void ui_main(){
        // Initialization
        //--------------------------------------------------------------------------------------
        int screenWidth = 1920;
        int screenHeight = 1080;
        SetTargetFPS(60);

        SetConfigFlags(FLAG_WINDOW_RESIZABLE);
        SetConfigFlags(FLAG_WINDOW_MAXIMIZED);
        SetConfigFlags(FLAG_MSAA_4X_HINT);
        InitWindow(screenWidth, screenHeight, "auto controller");
        screenWidth = GetScreenWidth();
        screenHeight = GetScreenHeight();

        float scale = 200;
        float map_draw_scale = 100;
        Vector2 map_draw_origin = {(float)screenWidth / 2, (float)screenHeight - 400};

        RobotChassis4 robot({300, 300}, scale, 0.3, 0.3 * 0.051, 0.5, RED);
        PoseAdjust pa(map_draw_origin, map_draw_scale);
        toggle_button pa_enable({screenWidth / 2 + 400, screenHeight - 200}, 250, 70, GRAY, BLUE, false, 0.5);
        label pa_enable_label("initial pose", {screenWidth / 2 + 400 + 10, screenHeight - 200 + 25}, 20, BLACK);
        push_button ip_reset({screenWidth / 2 + 400, screenHeight - 300}, 250, 70, GRAY, BLUE, false, 0.5);
        label ip_reset_label("IP reset", {screenWidth / 2 + 400 + 10, screenHeight - 300 + 25}, 20, BLACK);

        push_button req_left_map({screenWidth / 2 + 400, screenHeight - 500}, 250, 70, GRAY, BLUE, false, 0.5);
        label req_left_map_label("request left map", {screenWidth / 2 + 400 + 10, screenHeight - 500 + 25}, 20, BLACK);
        push_button req_right_map({screenWidth / 2 + 400, screenHeight - 400}, 250, 70, GRAY, BLUE, false, 0.5);
        label req_right_map_label("request right map", {screenWidth / 2 + 400 + 10, screenHeight - 400 + 25}, 20, BLACK);

        push_button ready_seq({screenWidth / 2 + 400, 100}, 250, 70, GRAY, BLUE, false, 0.5);
        label ready_seq_label("READY", {screenWidth / 2 + 400 + 10, 100 + 25}, 20, BLACK);
        push_button right_seq({screenWidth / 2 + 400, 200}, 250, 70, SKYBLUE, BLUE, false, 0.5);
        label right_seq_label("RIGHT", {screenWidth / 2 + 400 + 10, 200 + 25}, 20, BLACK);
        push_button left_seq({screenWidth / 2 + 400, 300}, 250, 70, PINK, RED, false, 0.5);
        label left_seq_label("LEFT", {screenWidth / 2 + 400 + 10, 300 + 25}, 20, BLACK);

        PathCallers path_callers(get_path_client_);
        std::vector<uint8_t> path_names;
        path_names.push_back(pure_pursuit::srv::GetPath::Request::RIGHT_START);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::RIGHT_DAIZA_COLLECT);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::RIGHT_DAIZA_PLACE);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::RIGHT_HINA_ZONE);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::RIGHT_HINA_COLLECT);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::RIGHT_HINA_PLACE);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::RIGHT_HINA_PLACE_RETRY);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::LEFT_START);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::LEFT_DAIZA_COLLECT);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::LEFT_DAIZA_PLACE);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::LEFT_HINA_ZONE);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::LEFT_HINA_COLLECT);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::LEFT_HINA_PLACE);
        path_names.push_back(pure_pursuit::srv::GetPath::Request::LEFT_HINA_PLACE_RETRY);
        BeginDrawing();
            ClearBackground(RAYWHITE);
            DrawText("Loading paths...", 10, 10, 20, GRAY);
        EndDrawing();
        path_callers.RequestPaths(path_names);
        while(!path_callers.isFinished() && !WindowShouldClose() && rclcpp::ok()){
            // rclcpp::spin_some(this->get_node_base_interface());
            BeginDrawing();
            ClearBackground(RAYWHITE);
            std::stringstream ss;
            ss << "Loading paths..." << std::endl;
            ss << "Loaded" << path_callers.getOkNum() << " / " << path_callers.getRequestNum() << std::endl;
            DrawText(ss.str().c_str(), 10, 10, 20, GRAY);
            EndDrawing();
        }
        

        RobotActionList ra_list({100, screenHeight - 600}, std::string("test"), &robot_actions_manager_);
        // add ready sequence
        size_t ready_seq_index_from = ra_list.size();
        {
            auto bonbori_off = mecha_control::msg::MechAction();
            bonbori_off.type = mecha_control::msg::MechAction::BONBORI;
            bonbori_off.bonbori_enable = false;
            ra_list.add_action(RobotAction(bonbori_off, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("bonbori off"));

            auto daiza_ready = mecha_control::msg::MechAction();
            daiza_ready.type = mecha_control::msg::MechAction::DAIZA;
            daiza_ready.daiza.command = mecha_control::msg::DaizaCmdType::READY;
            ra_list.add_action(RobotAction(daiza_ready, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza ready"));

            auto hina_ready = mecha_control::msg::MechAction();
            hina_ready.type = mecha_control::msg::MechAction::HINA;
            hina_ready.hina.command = mecha_control::msg::HinaCmdType::READY;
            ra_list.add_action(RobotAction(hina_ready, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina ready"));
        }
        size_t ready_seq_index_to = ra_list.size() - 1;
        // add right sequence
        size_t right_seq_index_from = ra_list.size();
        {
            auto bonbori_off = mecha_control::msg::MechAction();
            bonbori_off.type = mecha_control::msg::MechAction::BONBORI;
            bonbori_off.bonbori_enable = false;
            ra_list.add_action(RobotAction(bonbori_off, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("bonbori off"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::RIGHT_START),
                path_and_feedback_client_
            ), std::string("start"));

            auto daiza_unclamp = mecha_control::msg::MechAction();
            daiza_unclamp.type = mecha_control::msg::MechAction::DAIZA;
            daiza_unclamp.daiza.command = mecha_control::msg::DaizaCmdType::EXPAND_AND_UNCLAMP;
            ra_list.add_action(RobotAction(daiza_unclamp, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza unclamp"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::RIGHT_DAIZA_COLLECT),
                path_and_feedback_client_
            ), std::string("move to daiza"));

            auto daiza_take = mecha_control::msg::MechAction();
            daiza_take.type = mecha_control::msg::MechAction::DAIZA;
            daiza_take.daiza.command = mecha_control::msg::DaizaCmdType::CLAMP_AND_CONTRACT;
            ra_list.add_action(RobotAction(daiza_take, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza clamp"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::RIGHT_DAIZA_PLACE),
                path_and_feedback_client_
            ), std::string("place daiza"));

            auto daiza_place = mecha_control::msg::MechAction();
            daiza_place.type = mecha_control::msg::MechAction::DAIZA;
            daiza_place.daiza.command = mecha_control::msg::DaizaCmdType::EXPAND_AND_PLACE_AND_CONTRACT;
            ra_list.add_action(RobotAction(daiza_place, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza ready"));

            auto daiza_contract = mecha_control::msg::MechAction();
            daiza_contract.type = mecha_control::msg::MechAction::DAIZA;
            daiza_contract.daiza.command = mecha_control::msg::DaizaCmdType::READY;
            ra_list.add_action(RobotAction(daiza_contract, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza clamp"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::RIGHT_HINA_ZONE),
                path_and_feedback_client_
            ), std::string("hina zone"));

            auto hina_expand = mecha_control::msg::MechAction();
            hina_expand.type = mecha_control::msg::MechAction::HINA;
            hina_expand.hina.command = mecha_control::msg::HinaCmdType::DOWN_AND_TAKE;
            ra_list.add_action(RobotAction(hina_expand, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina expand"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::RIGHT_HINA_COLLECT),
                path_and_feedback_client_
            ), std::string("move to hina"));

            auto hina_take = mecha_control::msg::MechAction();
            hina_take.type = mecha_control::msg::MechAction::HINA;
            hina_take.hina.command = mecha_control::msg::HinaCmdType::UP_AND_CARRY;
            ra_list.add_action(RobotAction(hina_take, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina up"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::RIGHT_HINA_PLACE),
                path_and_feedback_client_
            ), std::string("place hina"));

            auto hina_place_pos = mecha_control::msg::MechAction();
            hina_place_pos.type = mecha_control::msg::MechAction::HINA;
            hina_place_pos.hina.command = mecha_control::msg::HinaCmdType::UP_AND_PLACE;
            ra_list.add_action(RobotAction(hina_place_pos, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina place pos"));

            auto hina_launch = mecha_control::msg::MechAction();
            hina_launch.type = mecha_control::msg::MechAction::HINA;
            hina_launch.hina.command = mecha_control::msg::HinaCmdType::LATCH_UNLOCK;
            ra_list.add_action(RobotAction(hina_launch, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina launch"));

            auto bonbori_on = mecha_control::msg::MechAction();
            bonbori_on.type = mecha_control::msg::MechAction::BONBORI;
            bonbori_on.bonbori_enable = true;
            ra_list.add_action(RobotAction(bonbori_on, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("bonbori on"));

            // auto hina_ready = mecha_control::msg::MechAction();
            // hina_ready.type = mecha_control::msg::MechAction::HINA;
            // hina_ready.hina.command = mecha_control::msg::HinaCmdType::READY;
            // ra_list.add_action(RobotAction(hina_ready, daiza_cmd_client_, hina_cmd_client_, bonbori_srv_client_), std::string("hina up"));
        }
        size_t right_seq_index_to = ra_list.size() - 1;
        // add left sequence
        size_t left_seq_index_from = ra_list.size();
        {
            auto bonbori_off = mecha_control::msg::MechAction();
            bonbori_off.type = mecha_control::msg::MechAction::BONBORI;
            bonbori_off.bonbori_enable = false;
            ra_list.add_action(RobotAction(bonbori_off, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("bonbori off"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::LEFT_START),
                path_and_feedback_client_
            ), std::string("start"));

            auto daiza_unclamp = mecha_control::msg::MechAction();
            daiza_unclamp.type = mecha_control::msg::MechAction::DAIZA;
            daiza_unclamp.daiza.command = mecha_control::msg::DaizaCmdType::EXPAND_AND_UNCLAMP;
            ra_list.add_action(RobotAction(daiza_unclamp, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza unclamp"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::LEFT_DAIZA_COLLECT),
                path_and_feedback_client_
            ), std::string("move to daiza"));

            auto daiza_take = mecha_control::msg::MechAction();
            daiza_take.type = mecha_control::msg::MechAction::DAIZA;
            daiza_take.daiza.command = mecha_control::msg::DaizaCmdType::CLAMP_AND_CONTRACT;
            ra_list.add_action(RobotAction(daiza_take, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza clamp"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::LEFT_DAIZA_PLACE),
                path_and_feedback_client_
            ), std::string("place daiza"));

            auto daiza_place = mecha_control::msg::MechAction();
            daiza_place.type = mecha_control::msg::MechAction::DAIZA;
            daiza_place.daiza.command = mecha_control::msg::DaizaCmdType::EXPAND_AND_PLACE_AND_CONTRACT;
            ra_list.add_action(RobotAction(daiza_place, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza ready"));

            auto daiza_contract = mecha_control::msg::MechAction();
            daiza_contract.type = mecha_control::msg::MechAction::DAIZA;
            daiza_contract.daiza.command = mecha_control::msg::DaizaCmdType::READY;
            ra_list.add_action(RobotAction(daiza_contract, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("daiza clamp"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::LEFT_HINA_ZONE),
                path_and_feedback_client_
            ), std::string("hina zone"));

            auto hina_expand = mecha_control::msg::MechAction();
            hina_expand.type = mecha_control::msg::MechAction::HINA;
            hina_expand.hina.command = mecha_control::msg::HinaCmdType::DOWN_AND_TAKE;
            ra_list.add_action(RobotAction(hina_expand, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina expand"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::LEFT_HINA_COLLECT),
                path_and_feedback_client_
            ), std::string("move to hina"));

            auto hina_take = mecha_control::msg::MechAction();
            hina_take.type = mecha_control::msg::MechAction::HINA;
            hina_take.hina.command = mecha_control::msg::HinaCmdType::UP_AND_CARRY;
            ra_list.add_action(RobotAction(hina_take, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina up"));

            ra_list.add_action(RobotAction(
                path_callers.getPath(pure_pursuit::srv::GetPath::Request::LEFT_HINA_PLACE),
                path_and_feedback_client_
            ), std::string("place hina"));

            auto hina_place_pos = mecha_control::msg::MechAction();
            hina_place_pos.type = mecha_control::msg::MechAction::HINA;
            hina_place_pos.hina.command = mecha_control::msg::HinaCmdType::UP_AND_PLACE;
            ra_list.add_action(RobotAction(hina_place_pos, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina place pos"));

            auto hina_launch = mecha_control::msg::MechAction();
            hina_launch.type = mecha_control::msg::MechAction::HINA;
            hina_launch.hina.command = mecha_control::msg::HinaCmdType::LATCH_UNLOCK;
            ra_list.add_action(RobotAction(hina_launch, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("hina launch"));

            auto bonbori_on = mecha_control::msg::MechAction();
            bonbori_on.type = mecha_control::msg::MechAction::BONBORI;
            bonbori_on.bonbori_enable = true;
            ra_list.add_action(RobotAction(bonbori_on, daiza_cmd_client_, hina_cmd_client_, bonbori_msg_pub_), std::string("bonbori on"));

            // auto hina_ready = mecha_control::msg::MechAction();
            // hina_ready.type = mecha_control::msg::MechAction::HINA;
            // hina_ready.hina.command = mecha_control::msg::HinaCmdType::READY;
            // ra_list.add_action(RobotAction(hina_ready, daiza_cmd_client_, hina_cmd_client_, bonbori_srv_client_), std::string("hina up"));
        }
        size_t left_seq_index_to = ra_list.size() - 1;
        ra_list.change_action_to_manager(left_seq_index_from, left_seq_index_to);
        ra_list.set_name(std::string("LEFT"));

        // slider sl({1000, 400}, {1100, 400}, 5.0f, BLUE);
        // toggle_button tb({1000, 300}, 100, 50, GRAY, BLUE);

        int count = 0;
        //--------------------------------------------------------------------------------------

        // Main game loop
        while (!WindowShouldClose() && rclcpp::ok())    // Detect window close button or ESC key
        {
            // Update
            //----------------------------------------------------------------------------------
            //resize action
            screenWidth = GetScreenWidth();
            screenHeight = GetScreenHeight();

            map_draw_origin = {(float)screenWidth / 2, (float)screenHeight - 400};
            robot.set_pos({300, 300});
            robot.set_vel(robot_vec_.x, robot_vec_.y, robot_vec_.z);
            robot.set_wheels_vel(input_vel_);
            pa.set_map_draw_origin(map_draw_origin);
            pa_enable.move({screenWidth / 2 + 400, screenHeight - 200});
            pa_enable_label.move({screenWidth / 2 + 400 + 10, screenHeight - 200 + 25});
            ip_reset.move({screenWidth / 2 + 400, screenHeight - 300});
            ip_reset_label.move({screenWidth / 2 + 400 + 10, screenHeight - 300 + 25});
            req_left_map.move({screenWidth / 2 + 400, screenHeight - 500});
            req_left_map_label.move({screenWidth / 2 + 400 + 10, screenHeight - 500 + 25});
            req_right_map.move({screenWidth / 2 + 400, screenHeight - 400});
            req_right_map_label.move({screenWidth / 2 + 400 + 10, screenHeight - 400 + 25});
            ready_seq.move({screenWidth / 2 + 400, 100});
            ready_seq_label.move({screenWidth / 2 + 400 + 10, 100 + 25});
            right_seq.move({screenWidth / 2 + 400, 200});
            right_seq_label.move({screenWidth / 2 + 400 + 10, 200 + 25});
            left_seq.move({screenWidth / 2 + 400, 300});
            left_seq_label.move({screenWidth / 2 + 400 + 10, 300 + 25});
            ra_list.move({100, screenHeight - 600});

            Vector2 mouse = GetMousePosition();
            pa_enable.process(mouse);
            ip_reset.process(mouse);
            req_left_map.process(mouse);
            req_right_map.process(mouse);
            ready_seq.process(mouse);
            right_seq.process(mouse);
            left_seq.process(mouse);
            ra_list.process(mouse, switchs_[0]);

            if(pa_enable.get_value() && !CheckCollisionPointRec(mouse, pa_enable.get_rec())){
                if (pa.process(mouse)){
                    auto pose = pa.get_pose();
                    mouse_point_pub_->publish(pose);
                    pa_enable.set_value(false);
                }
            }

            if(req_left_map.is_pressed()){
                cclp::msg::MapRequest req;
                req.map = cclp::msg::MapRequest::MAP_LEFT;
                map_request_pub_->publish(req);
            }
            if(req_right_map.is_pressed()){
                cclp::msg::MapRequest req;
                req.map = cclp::msg::MapRequest::MAP_RIGHT;
                map_request_pub_->publish(req);
            }
            
            if(ready_seq.is_pressed()){
                ra_list.change_action_to_manager(ready_seq_index_from, ready_seq_index_to);
                ra_list.set_name(std::string("READY"));
            }
            if(right_seq.is_pressed()){
                ra_list.change_action_to_manager(right_seq_index_from, right_seq_index_to);
                ra_list.set_name(std::string("RIGHT"));
            }
            if(left_seq.is_pressed()){
                ra_list.change_action_to_manager(left_seq_index_from, left_seq_index_to);
                ra_list.set_name(std::string("LEFT"));
            }


            Vector3 tf_vec3;
            std::string err_tf;
            geometry_msgs::msg::TransformStamped transform;
            try{
                transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
                Quaternion q = {transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w};
                tf_vec3 = {transform.transform.translation.x, transform.transform.translation.y, QuaternionToEuler(q).z};
            }catch(tf2::TransformException &ex){
                err_tf = ex.what();
            }
            Vector3 tf_vec3_base;
            std::string err_tf_base;
            try{
                geometry_msgs::msg::TransformStamped transform_base = tf_buffer_->lookupTransform(map_frame_, "base_link", tf2::TimePointZero);
                Quaternion q_base = {transform_base.transform.rotation.x, transform_base.transform.rotation.y, transform_base.transform.rotation.z, transform_base.transform.rotation.w};
                tf_vec3_base = {transform_base.transform.translation.x, transform_base.transform.translation.y, QuaternionToEuler(q_base).z};
            }catch(tf2::TransformException &ex){
                err_tf_base = ex.what();
            }
            std::vector<Vector2> moved_points1;
            {
                std::lock_guard<std::mutex> lock(points1_mutex);
                moved_points1 = points1;
            }
            move_points(moved_points1, tf2d_from_vec3(tf_vec3));
            std::vector<Vector2> moved_points2;
            {
                std::lock_guard<std::mutex> lock(points2_mutex);
                moved_points2 = points2;
            }
            move_points(moved_points2, tf2d_from_vec3(tf_vec3));

            if(ip_reset.is_pressed()){
                auto pose = geometry_msgs::msg::Pose();
                pose.position.x = transform.transform.translation.x;
                pose.position.y = transform.transform.translation.y;
                pose.position.z = transform.transform.translation.z;
                pose.orientation = transform.transform.rotation;
                mouse_point_pub_->publish(pose);
            }

            robot_actions_manager_.process();

            // sl.process(mouse);
            // tb.process(mouse);

            // Draw
            //----------------------------------------------------------------------------------
            BeginDrawing();

                ClearBackground(RAYWHITE);

                // sl.draw();
                // tb.draw();

                // Draw the points
                draw_points_scale_y_inv(moved_points1, map_draw_scale, map_draw_origin, BLUE);
                draw_points_scale_y_inv(moved_points2, map_draw_scale, map_draw_origin, BLUE);
                // Draw the lines
                {
                    std::lock_guard<std::mutex> lock(lines_mutex);
                    draw_lines_scale_y_inv(lines, map_draw_scale, map_draw_origin);
                }
                path_ = robot_actions_manager_.getCurrentPath();
                draw_lines_scale_y_inv(path_, map_draw_scale, map_draw_origin, GREEN);
                // Draw the transform
                draw_tf_scale_y_inv(tf_vec3, map_draw_scale, map_draw_origin);
                draw_tf_scale_y_inv(tf_vec3_base, map_draw_scale, map_draw_origin);

                robot.draw_y_inv();
                pa.draw();
                pa_enable.draw();
                pa_enable_label.draw();
                ip_reset.draw();
                ip_reset_label.draw();
                req_left_map.draw();
                req_left_map_label.draw();
                req_right_map.draw();
                req_right_map_label.draw();
                ready_seq.draw();
                ready_seq_label.draw();
                right_seq.draw();
                right_seq_label.draw();
                left_seq.draw();
                left_seq_label.draw();
                ra_list.draw();

                std::stringstream ss;
                ss << "FPS" << GetFPS() << std::endl << std::endl;
                ss << "Transform: " << tf_vec3.x << ", " << tf_vec3.y << ", " << tf_vec3.z << std::endl;
                ss << "Points1: " << moved_points1.size() << std::endl;
                ss << "Points2: " << moved_points2.size() << std::endl;
                ss << "Lines: " << lines.size() << std::endl;
                ss << "Error transform: " << err_tf << std::endl;
                ss << "Error base transorm: " << err_tf_base << std::endl;
                DrawText(ss.str().c_str(), 10, 10, 20, GRAY);
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

    void input_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if(msg->data.size() != 4){
            RCLCPP_ERROR(this->get_logger(), "Invalid input_vel message size");
            return;
        }
        for(int i = 0; i < 4; i++){
            input_vel_[i] = msg->data[i];
        }
    }

    void robot_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        robot_vec_ = {msg->linear.x, msg->linear.y, msg->angular.z};
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_.clear();
        path_.reserve(msg->poses.size() - 1);
        for(unsigned int i = 0; i < msg->poses.size() - 1; i++){
            Vector2 from = {msg->poses[i].pose.position.x, msg->poses[i].pose.position.y};
            Vector2 to = {msg->poses[i + 1].pose.position.x, msg->poses[i + 1].pose.position.y};
            path_.push_back({from, to});
        }
    }

   void Laser1ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::lock_guard<std::mutex> lock(points1_mutex);
        points1.clear();
        points1.reserve(msg->ranges.size());
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            if(std::isinf(msg->ranges[i])) continue;
            if(std::isnan(msg->ranges[i])) continue;
            float angle = msg->angle_min + msg->angle_increment * i + lidar1_offset_theta_;
            float x = msg->ranges[i] * std::cos(angle) + lidar1_offset_x_;
            float y = msg->ranges[i] * std::sin(angle) + lidar1_offset_y_;
            if(lidar1_invert_x_) x = -x;
            if(std::pow(x - lidar1_circle_mask_center_x_, 2) + std::pow(y - lidar1_circle_mask_center_y_, 2) < std::pow(lidar1_circle_mask_radius_, 2)) continue;
            points1.push_back({x, y});
        }
    }

    void Laser2ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::lock_guard<std::mutex> lock(points2_mutex);
        points2.clear();
        points2.reserve(msg->ranges.size());
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            if(std::isinf(msg->ranges[i])) continue;
            if(std::isnan(msg->ranges[i])) continue;
            float angle = msg->angle_min + msg->angle_increment * i + lidar2_offset_theta_;
            float x = msg->ranges[i] * std::cos(angle) + lidar2_offset_x_;
            float y = msg->ranges[i] * std::sin(angle) + lidar2_offset_y_;
            if(lidar2_invert_x_) x = -x;
            if(std::pow(x - lidar2_circle_mask_center_x_, 2) + std::pow(y - lidar2_circle_mask_center_y_, 2) < std::pow(lidar2_circle_mask_radius_, 2)) continue;
            points2.push_back({x, y});
        }
    }

    void line_map_callback(const cclp::msg::LineArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(lines_mutex);
        lines.clear();
        lines.reserve(msg->lines.size());
        for (auto line : msg->lines)
        {
            Line l;
            l.from = {line.p1.x, line.p1.y};
            l.to = {line.p2.x, line.p2.y};
            lines.push_back(l);
        }
        RCLCPP_INFO(this->get_logger(), "Received %d lines", lines.size());
    }

    void switchs_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg){
        if(msg->data.size() != 4){
            RCLCPP_ERROR(this->get_logger(), "Invalid switchs message size");
            return;
        }
        for(int i = 0; i < 4; i++){
            switchs_[i] = msg->data[i];
        }
    }
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoController>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}