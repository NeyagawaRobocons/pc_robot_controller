#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "pure_pursuit/msg/path2_d_with_speed.hpp"
#include "mecha_control/msg/mech_action.hpp"
#include "pure_pursuit/srv/get_path.hpp"
#include "pure_pursuit/action/path_and_feedback.hpp"
#include "mecha_control/action/daiza_cmd.hpp"
#include "mecha_control/action/hina_cmd.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cclp/include/lines_and_points.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

class RobotAction
{
public:
    enum class ActionType
    {
        MOVE_PATH,
        MOVE_PATH_WITH_MECH_ACTION,
        MECH_ACTION,
        MECH_ACTION_NON_BLOCKING,
    };
    struct MechActionPath
    {
        std::vector<mecha_control::msg::MechAction> mech_actions_path;
        std::vector<int32_t> mech_action_indices;
    };
    enum class ExecuteResult
    {
        SUCCESS,
        FAILED_NO_ACTION_SERVER,
        FAILED_TIMEOUT,
        FAILED_CANNOT_EXECUTE,
    };
// private:
    ActionType action_type_;
    pure_pursuit::msg::Path2DWithParams path_;
    MechActionPath mech_action_path_;
    mecha_control::msg::MechAction mech_action_;
    int32_t path_current_index_ = 0;
    std::string status_;
    rclcpp_action::GoalUUID move_path_goal_uuid_;
    rclcpp_action::GoalUUID daiza_cmd_goal_uuid_;
    rclcpp_action::GoalUUID hina_cmd_goal_uuid_;
    int64_t bonbori_srv_request_id_;
    bool finished_ = true;

    std::string move_path_action_name_ = "path_and_feedback";
    std::string daiza_cmd_action_name_ = "daiza_cmd";
    std::string hina_cmd_action_name_ = "hina_cmd";
    std::string bonbori_srv_name_ = "set_bonbori";
    rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client_;
    rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client_;
    rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bonbori_pub_;
    // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client_;
public:
    RobotAction(pure_pursuit::msg::Path2DWithParams path, rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client) {
        action_type_ = ActionType::MOVE_PATH;
        path_ = path;
        move_path_action_client_ = move_path_action_client;
    };
    RobotAction(pure_pursuit::msg::Path2DWithParams path, MechActionPath mech_action_path,
        rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client,
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bonbori_pub
        // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client
    ) {
        action_type_ = ActionType::MOVE_PATH_WITH_MECH_ACTION;
        path_ = path;
        mech_action_path_ = mech_action_path;
        move_path_action_client_ = move_path_action_client;
        daiza_cmd_action_client_ = daiza_cmd_action_client;
        hina_cmd_action_client_ = hina_cmd_action_client;
        bonbori_pub_ = bonbori_pub;
        // bonbori_srv_client_ = bonbori_srv_client;
    };
    RobotAction(mecha_control::msg::MechAction mech_action, 
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client,
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bonbori_pub,
        // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client,
        bool non_blocking=false
    ) {
        action_type_ = non_blocking ? ActionType::MECH_ACTION_NON_BLOCKING : ActionType::MECH_ACTION;
        mech_action_ = mech_action;
        daiza_cmd_action_client_ = daiza_cmd_action_client;
        hina_cmd_action_client_ = hina_cmd_action_client;
        bonbori_pub_ = bonbori_pub;
        // bonbori_srv_client_ = bonbori_srv_client;
    };
    RobotAction(ActionType action_type, pure_pursuit::msg::Path2DWithParams path, MechActionPath mech_action_path, mecha_control::msg::MechAction mech_action,
        rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client,
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bonbori_pub
        // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client
    ) {
        action_type_ = action_type;
        path_ = path;
        mech_action_path_ = mech_action_path;
        mech_action_ = mech_action;
        move_path_action_client_ = move_path_action_client;
        daiza_cmd_action_client_ = daiza_cmd_action_client;
        hina_cmd_action_client_ = hina_cmd_action_client;
        bonbori_pub_ = bonbori_pub;
        // bonbori_srv_client_ = bonbori_srv_client;
    };
    ActionType getActionType() {
        return action_type_;
    };
    void execute(rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SendGoalOptions move_path_send_goal_options,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options
    ) {
        finished_ = false;
        switch (action_type_) {
            case ActionType::MOVE_PATH:
                executeMovePath(move_path_send_goal_options);
                break;
            case ActionType::MOVE_PATH_WITH_MECH_ACTION:
                executeMovePathWithMechAction(move_path_send_goal_options);
                break;
            case ActionType::MECH_ACTION:
            case ActionType::MECH_ACTION_NON_BLOCKING:
                executeMechAction(daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
                break;
        }
    };
    void abort() {
        if(action_type_ == ActionType::MOVE_PATH){
            move_path_action_client_->async_cancel_all_goals();
        } else if(action_type_ == ActionType::MOVE_PATH_WITH_MECH_ACTION){
            move_path_action_client_->async_cancel_all_goals();
            daiza_cmd_action_client_->async_cancel_all_goals();
            hina_cmd_action_client_->async_cancel_all_goals();
        } else if(action_type_ == ActionType::MECH_ACTION){
            daiza_cmd_action_client_->async_cancel_all_goals();
            hina_cmd_action_client_->async_cancel_all_goals();
        }
    };
    std::string getStatus() {
        // std::cout << "status_:" << status_ << std::endl;
        return status_;
    };
    bool isFinished() {
        return finished_;
    };

    std::function<void(rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::SharedPtr, const std::shared_ptr<const pure_pursuit::action::PathAndFeedback::Feedback>, rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions, rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions)> get_move_path_feedback_callback() {
        if(action_type_ == ActionType::MOVE_PATH){
            return std::bind(&RobotAction::executeMovePath_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        } else if(action_type_ == ActionType::MOVE_PATH_WITH_MECH_ACTION){
            return std::bind(&RobotAction::executeMovePathWithMechAction_feedback_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        } else {
            return [](rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::SharedPtr, const std::shared_ptr<const pure_pursuit::action::PathAndFeedback::Feedback>, rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions, rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions){};
        }
    }
    std::function<void(const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult &, rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions, rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions)> get_move_path_result_callback() {
        if(action_type_ == ActionType::MOVE_PATH){
            return std::bind(&RobotAction::executeMovePath_result_callback, this, std::placeholders::_1);
        } else if(action_type_ == ActionType::MOVE_PATH_WITH_MECH_ACTION){
            return std::bind(&RobotAction::executeMovePathWithMechAction_result_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        } else {
            return [](const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult &, rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions, rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions){};
        }
    }
    std::function<void(rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::SharedPtr, const std::shared_ptr<const mecha_control::action::DaizaCmd::Feedback>)> get_daiza_feedback_callback() {
        if(action_type_ == ActionType::MECH_ACTION){
            return std::bind(&RobotAction::daiza_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        } else {
            return [](rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::SharedPtr, const std::shared_ptr<const mecha_control::action::DaizaCmd::Feedback>){};
        }
    }
    std::function<void(const rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::WrappedResult &)> get_daiza_result_callback() {
        if(action_type_ == ActionType::MECH_ACTION){
            return std::bind(&RobotAction::daiza_result_callback, this, std::placeholders::_1);
        } else {
            return [](const rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::WrappedResult &){};
        }
    }
    std::function<void(rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::SharedPtr, const std::shared_ptr<const mecha_control::action::HinaCmd::Feedback>)> get_hina_feedback_callback() {
        if(action_type_ == ActionType::MECH_ACTION){
            return std::bind(&RobotAction::hina_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        } else {
            return [](rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::SharedPtr, const std::shared_ptr<const mecha_control::action::HinaCmd::Feedback>){};
        }
    }
    std::function<void(const rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::WrappedResult &)> get_hina_result_callback() {
        if(action_type_ == ActionType::MECH_ACTION){
            return std::bind(&RobotAction::hina_result_callback, this, std::placeholders::_1);
        } else {
            return [](const rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::WrappedResult &){};
        }
    }

private:
    rclcpp_action::GoalUUID executeMovePath(rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SendGoalOptions send_goal_options){
        if (!move_path_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            status_ = "Action server not available";
            return rclcpp_action::GoalUUID();
        }
        auto goal_msg = pure_pursuit::action::PathAndFeedback::Goal();
        goal_msg.path = path_;
        goal_msg.feedback_indices = mech_action_path_.mech_action_indices;
        auto goal_handle_future = move_path_action_client_->async_send_goal(goal_msg, send_goal_options);
        move_path_goal_uuid_ = goal_handle_future.get()->get_goal_id();
        return move_path_goal_uuid_;
    }
    void executeMovePath_feedback_callback(rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::SharedPtr goal_handle, const std::shared_ptr<const pure_pursuit::action::PathAndFeedback::Feedback> feedback){
        if(goal_handle->get_goal_id() == move_path_goal_uuid_){
            std::stringstream ss;
            path_current_index_ = feedback->current_index;
            ss << "Feedback: " << path_current_index_;
            status_ = ss.str();
        }
    }
    void executeMovePath_result_callback(const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult & result){
        if(result.goal_id == move_path_goal_uuid_){
            std::stringstream ss;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    ss << "Goal";
                    ss << " with " << result.result->final_index;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    ss << "Goal was aborted";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    ss << "Goal was canceled";
                    break;
                default:
                    ss << "Unknown result code";
                    break;
            }
            status_ = ss.str();
            finished_ = true;
        }
    }
    rclcpp_action::GoalUUID executeMovePathWithMechAction(rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SendGoalOptions move_path_send_goal_options
    ) {
        if (!move_path_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            status_ = "Action server not available";
            return rclcpp_action::GoalUUID();
        }
        auto move_path_goal_msg = pure_pursuit::action::PathAndFeedback::Goal();
        move_path_goal_msg.path = path_;
        move_path_goal_msg.feedback_indices = mech_action_path_.mech_action_indices;
        auto move_path_goal_handle_future = move_path_action_client_->async_send_goal(move_path_goal_msg, move_path_send_goal_options);
        move_path_goal_uuid_ = move_path_goal_handle_future.get()->get_goal_id();
        return move_path_goal_uuid_;
    }
    void executeMovePathWithMechAction_feedback_callback(rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::SharedPtr goal_handle,
        const std::shared_ptr<const pure_pursuit::action::PathAndFeedback::Feedback> feedback,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options
    ){
        std::cout << "executeMovePathWithMechAction_feedback_callback" << std::endl;
        std::cout << "goal_handle->get_goal_id():";
        for (auto i : goal_handle->get_goal_id()) {
            std::cout << std::hex << i;
        }
        std::cout << std::endl;
        std::cout << "move_path_goal_uuid_:";
        for (auto i : move_path_goal_uuid_) {
            std::cout << std::hex << i;
        }
        std::cout << std::endl;
        if(goal_handle->get_goal_id() == move_path_goal_uuid_){
            std::stringstream ss;
            path_current_index_ = feedback->current_index;
            ss << "Feedback: " << path_current_index_;
            status_ = ss.str();
            // for(auto i : mech_action_path_.mech_action_indices){
            //     if(path_current_index_ >= i){
            //         mech_action_ = mech_action_path_.mech_actions_path[i];
            //         executeMechAction_no_callback(daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
            //         mech_action_path_.mech_action_indices.erase(mech_action_path_.mech_action_indices.begin());
            //         mech_action_path_.mech_actions_path.erase(mech_action_path_.mech_actions_path.begin());
            //     }
            // }
        }
    }
    void executeMovePathWithMechAction_result_callback(const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult & result,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options
        ){
        if(result.goal_id == move_path_goal_uuid_){
            std::stringstream ss;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    ss << "Goal";
                    ss << " with " << result.result->final_index;
                    // for(auto i : mech_action_path_.mech_action_indices){
                    //     if(path_current_index_ >= i){
                    //         mech_action_ = mech_action_path_.mech_actions_path[i];
                    //         executeMechAction_no_callback(daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
                    //         mech_action_path_.mech_action_indices.erase(mech_action_path_.mech_action_indices.begin());
                    //         mech_action_path_.mech_actions_path.erase(mech_action_path_.mech_actions_path.begin());
                    //     }
                    //     if(i > result.result->final_index){
                    //         ss << " but cannot execute " << mech_action_path_.mech_action_indices.size() <<" mech action";
                    //         break;
                    //     }
                    // }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    ss << "Goal was aborted";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    ss << "Goal was canceled";
                    break;
                default:
                    ss << "Unknown result code";
                    break;
            }
            status_ = ss.str();
            finished_ = true;
        }
    }
    rclcpp_action::GoalUUID executeMechAction(rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options
    ) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction called");
        rclcpp_action::GoalUUID goal_uuid;
        if (!daiza_cmd_action_client_->wait_for_action_server(std::chrono::seconds(1)) || !hina_cmd_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            status_ = "Action server not available";
            return goal_uuid;
        }
        if(mech_action_.type == mecha_control::msg::MechAction::DAIZA){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction DAIZA");
            auto daiza_cmd_goal_msg = mecha_control::action::DaizaCmd::Goal();
            daiza_cmd_goal_msg.command = mech_action_.daiza.command;
            auto daiza_cmd_goal_handle_future = daiza_cmd_action_client_->async_send_goal(daiza_cmd_goal_msg, daiza_cmd_send_goal_options);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction DAIZA sended");
            this->daiza_cmd_goal_uuid_ = daiza_cmd_goal_handle_future.get()->get_goal_id();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction get goal id");
            goal_uuid = daiza_cmd_goal_uuid_;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction DAIZA uuid : %d", daiza_cmd_goal_uuid_);
        } else if(mech_action_.type == mecha_control::msg::MechAction::HINA){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction HINA");
            auto hina_cmd_goal_msg = mecha_control::action::HinaCmd::Goal();
            hina_cmd_goal_msg.command = mech_action_.hina.command;
            auto hina_cmd_goal_handle_future = hina_cmd_action_client_->async_send_goal(hina_cmd_goal_msg, hina_cmd_send_goal_options);
            this->hina_cmd_goal_uuid_ = hina_cmd_goal_handle_future.get()->get_goal_id();
            goal_uuid = hina_cmd_goal_uuid_;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction HINA uuid : %d", hina_cmd_goal_uuid_);
        } else if(mech_action_.type == mecha_control::msg::MechAction::BONBORI){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction BONBORI");
            // auto bonbori_srv_request = std_srvs::srv::SetBool::Request::SharedPtr();
            // bonbori_srv_request->data = mech_action_.bonbori_enable;
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction send BONBORI request");
            // auto bonbori_srv_future = bonbori_srv_client_->async_send_request(bonbori_srv_request, std::bind(&RobotAction::bonbori_result_callback, this, std::placeholders::_1));
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction BONBORI sended");
            auto bonbori_pub_msg = std_msgs::msg::Bool();
            bonbori_pub_msg.data = mech_action_.bonbori_enable;
            bonbori_pub_->publish(bonbori_pub_msg);
            finished_ = true;
        }
        if(action_type_ == ActionType::MECH_ACTION_NON_BLOCKING){
            finished_ = true;
        }
        status_ = "Executing mech action";
        return goal_uuid;
    }

    rclcpp_action::GoalUUID executeMechAction_no_callback(rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options
    ) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction_no_callback called");
        rclcpp_action::GoalUUID goal_uuid;
        if (!daiza_cmd_action_client_->wait_for_action_server(std::chrono::seconds(1)) || !hina_cmd_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            status_ = "Action server not available";
            return goal_uuid;
        }
        if(mech_action_.type == mecha_control::msg::MechAction::DAIZA){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction DAIZA");
            auto daiza_cmd_goal_msg = mecha_control::action::DaizaCmd::Goal();
            daiza_cmd_goal_msg.command = mech_action_.daiza.command;
            rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options_empty;
            daiza_cmd_send_goal_options_empty.feedback_callback = std::bind([](rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::SharedPtr goal_handle, const std::shared_ptr<const mecha_control::action::DaizaCmd::Feedback> feedback){}, std::placeholders::_1, std::placeholders::_2);
            daiza_cmd_send_goal_options_empty.result_callback = std::bind([](const rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::WrappedResult & result){}, std::placeholders::_1);
            auto daiza_cmd_goal_handle_future = daiza_cmd_action_client_->async_send_goal(daiza_cmd_goal_msg, daiza_cmd_send_goal_options_empty);
            this->daiza_cmd_goal_uuid_ = daiza_cmd_goal_handle_future.get()->get_goal_id();
            goal_uuid = daiza_cmd_goal_uuid_;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction DAIZA uuid : %d", daiza_cmd_goal_uuid_);
        } else if(mech_action_.type == mecha_control::msg::MechAction::HINA){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction HINA");
            auto hina_cmd_goal_msg = mecha_control::action::HinaCmd::Goal();
            hina_cmd_goal_msg.command = mech_action_.hina.command;
            rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options_empty;
            hina_cmd_send_goal_options_empty.feedback_callback = std::bind([](rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::SharedPtr goal_handle, const std::shared_ptr<const mecha_control::action::HinaCmd::Feedback> feedback){}, std::placeholders::_1, std::placeholders::_2);
            hina_cmd_send_goal_options_empty.result_callback = std::bind([](const rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::WrappedResult & result){}, std::placeholders::_1);
            auto hina_cmd_goal_handle_future = hina_cmd_action_client_->async_send_goal(hina_cmd_goal_msg, hina_cmd_send_goal_options_empty);
            this->hina_cmd_goal_uuid_ = hina_cmd_goal_handle_future.get()->get_goal_id();
            goal_uuid = hina_cmd_goal_uuid_;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction HINA uuid : %d", hina_cmd_goal_uuid_);
        } else if(mech_action_.type == mecha_control::msg::MechAction::BONBORI){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeMechAction BONBORI");
            auto bonbori_pub_msg = std_msgs::msg::Bool();
            bonbori_pub_msg.data = mech_action_.bonbori_enable;
            bonbori_pub_->publish(bonbori_pub_msg);
            finished_ = true;
            // auto bonbori_srv_request = std_srvs::srv::SetBool::Request::SharedPtr();
            // bonbori_srv_request->data = mech_action_.bonbori_enable;
        }
        if(action_type_ == ActionType::MECH_ACTION_NON_BLOCKING){
            finished_ = true;
        }
        status_ = "Executing mech action";
        return goal_uuid;
    }
    
    void daiza_feedback_callback(rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::SharedPtr goal_handle,
        const std::shared_ptr<const mecha_control::action::DaizaCmd::Feedback> feedback
    ){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Daiza feedback callback");
        if(goal_handle->get_goal_id() == daiza_cmd_goal_uuid_){
            std::stringstream ss;
            ss << "Feedback: ";
            switch (feedback->feedback) {
                case mecha_control::action::DaizaCmd::Feedback::STOPPING:
                    ss << "STOPPING";
                    break;
                case mecha_control::action::DaizaCmd::Feedback::PREV_ACTION_ABORTED:
                    ss << "PREV_ACTION_ABORTED";
                    break;
                case mecha_control::action::DaizaCmd::Feedback::EXPANDED_AT_EXPAND_AND_UNCLAMP:
                    ss << "EXPANDED_AT_EXPAND_AND_UNCLAMP";
                    break;
                case mecha_control::action::DaizaCmd::Feedback::CLAMPED_AT_CLAMP_AND_CONTRACT:
                    ss << "CLAMPED_AT_CLAMP_AND_CONTRACT";
                    break;
                case mecha_control::action::DaizaCmd::Feedback::EXPANDED_AT_EXPAND_AND_PLACE:
                    ss << "EXPANDED_AT_EXPAND_AND_PLACE";
                    break;
                case mecha_control::action::DaizaCmd::Feedback::EXPANDED_AT_EXPAND_AND_PLACE_AND_CONTRACT:
                    ss << "EXPANDED_AT_EXPAND_AND_PLACE_AND_CONTRACT";
                    break;
                case mecha_control::action::DaizaCmd::Feedback::PLACE_AT_EXPAND_AND_PLACE_AND_CONTRACT:
                    ss << "PLACE_AT_EXPAND_AND_PLACE_AND_CONTRACT";
                    break;
            }
            status_ = ss.str();
        }
    }
    void daiza_result_callback(const rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::WrappedResult & result){
        if(result.goal_id == daiza_cmd_goal_uuid_){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Daiza result callback");
            std::stringstream ss;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    ss << "Goal was successful";
                    switch (result.result->result) {
                        case mecha_control::action::DaizaCmd::Result::OK:
                            ss << " with OK";
                            break;
                        case mecha_control::action::DaizaCmd::Result::ABORTED:
                            ss << " with ABORTED";
                            break;
                        case mecha_control::action::DaizaCmd::Result::ERR_UNEXPECTED_ARG:
                            ss << " with ERR_UNEXPECTED_ARG";
                            break;
                        case mecha_control::action::DaizaCmd::Result::ERR_CANNOT_PROCESS:
                            ss << " with ERR_CANNOT_PROCESS";
                            break;
                        case mecha_control::action::DaizaCmd::Result::ERR_TIMEOUT:
                            ss << " with ERR_TIMEOUT";
                            break;
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    ss << "Goal was aborted";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    ss << "Goal was canceled";
                    break;
                default:
                    ss << "Unknown result code";
                    break;
            }
            status_ = ss.str();
            finished_ = true;
        }
    }
    void hina_feedback_callback(rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::SharedPtr goal_handle,
        const std::shared_ptr<const mecha_control::action::HinaCmd::Feedback> feedback
    ){
        if(goal_handle->get_goal_id() == hina_cmd_goal_uuid_){
            std::stringstream ss;
            ss << "Feedback: ";
            switch (feedback->feedback) {
                case mecha_control::action::HinaCmd::Feedback::STOPPING:
                    ss << "STOPPING";
                    break;
                case mecha_control::action::HinaCmd::Feedback::PREV_ACTION_ABORTED:
                    ss << "PREV_ACTION_ABORTED";
                    break;
                case mecha_control::action::HinaCmd::Feedback::DOWN_AT_DOWN_AND_TAKE:
                    ss << "DOWN_AT_DOWN_AND_TAKE";
                    break;
                case mecha_control::action::HinaCmd::Feedback::UP_AT_UP_AND_CARRY:
                    ss << "UP_AT_UP_AND_CARRY";
                    break;
                case mecha_control::action::HinaCmd::Feedback::UP_AT_UP_AND_PLACE:
                    ss << "UP_AT_UP_AND_PLACE";
                    break;
            }
            status_ = ss.str();
        }
    }
    void hina_result_callback(const rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::WrappedResult & result){
        if(result.goal_id == hina_cmd_goal_uuid_){
            std::stringstream ss;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    ss << "Goal was successful";
                    switch (result.result->result) {
                        case mecha_control::action::HinaCmd::Result::OK:
                            ss << " with OK";
                            break;
                        case mecha_control::action::HinaCmd::Result::ABORTED:
                            ss << " with ABORTED";
                            break;
                        case mecha_control::action::HinaCmd::Result::ERR_UNEXPECTED_ARG:
                            ss << " with ERR_UNEXPECTED_ARG";
                            break;
                        case mecha_control::action::HinaCmd::Result::ERR_CANNOT_PROCESS:
                            ss << " with ERR_CANNOT_PROCESS";
                            break;
                        case mecha_control::action::HinaCmd::Result::ERR_TIMEOUT:
                            ss << " with ERR_TIMEOUT";
                            break;
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    ss << "Goal was aborted";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    ss << "Goal was canceled";
                    break;
                default:
                    ss << "Unknown result code";
                    break;
            }
            status_ = ss.str();
            finished_ = true;
        }
    }
    // void bonbori_result_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "bonbori_result_callback");
    //     std::stringstream ss;
    //     switch (future.get()->success) {
    //         case true:
    //             ss << "Bonbori :" << mech_action_.bonbori_enable;
    //             break;
    //         case false:
    //             ss << "Bonbori faled";
    //             break;
    //     }
    //     status_ = ss.str();
    //     finished_ = true;
    // }

};

class RobotActionsManager
{
public:
    RobotActionsManager(rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client,
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bonbori_pub
        // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client
    ) : move_path_action_client_(move_path_action_client), daiza_cmd_action_client_(daiza_cmd_action_client), hina_cmd_action_client_(hina_cmd_action_client), bonbori_pub_(bonbori_pub) {
    };
    size_t addRobotAction(RobotAction robot_action) {
        robot_actions_.push_back(robot_action);
        return robot_actions_.size() - 1;
    };
    void executeRobotAction(size_t index) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeRobotAction: %d", index);
        if(index < robot_actions_.size()){
            auto robot_action = &robot_actions_[index];
            auto move_path_send_goal_options = rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SendGoalOptions();
            auto daiza_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions();
            auto hina_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions();
            daiza_cmd_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::daiza_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            daiza_cmd_send_goal_options.result_callback = std::bind(&RobotActionsManager::daiza_result_callback, this, std::placeholders::_1);
            hina_cmd_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::hina_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            hina_cmd_send_goal_options.result_callback = std::bind(&RobotActionsManager::hina_result_callback, this, std::placeholders::_1);
            move_path_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::move_path_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            move_path_send_goal_options.result_callback = std::bind(&RobotActionsManager::move_path_result_callback, this, std::placeholders::_1);
            robot_action->execute(move_path_send_goal_options, daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
            robot_actions_index_ = index;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executeRobotAction: %d", robot_actions_index_);
            // robot_actions_[robot_actions_index_].move_path_goal_uuid_ = robot_action->move_path_goal_uuid_;
            // robot_actions_[robot_actions_index_].daiza_cmd_goal_uuid_ = robot_action->daiza_cmd_goal_uuid_;
            // robot_actions_[robot_actions_index_].hina_cmd_goal_uuid_ = robot_action->hina_cmd_goal_uuid_;
            // robot_actions_[robot_actions_index_].bonbori_srv_request_id_ = robot_action->bonbori_srv_request_id_;
            // robot_actions_[robot_actions_index_].finished_ = robot_action->finished_;
            // robot_actions_[robot_actions_index_].status_ = robot_action->status_;
            // running_robot_action_ = robot_action;
        }
    };
    void executeRobotActions(size_t index_from=0) {
        if(robot_actions_.size() > 0){
            robot_actions_index_ = index_from;
            executeRobotAction(robot_actions_index_);
        }
        running_executeRobotActions_ = true;
    };
    void executeRobotActions(size_t index_from, size_t index_to) {
        robot_actions_index_to_ = index_to;
        if(robot_actions_.size() > 0){
            robot_actions_index_ = index_from;
            executeRobotAction(robot_actions_index_);
        }
        running_executeRobotActions_ = true;
    };
    void process() {
        if(robot_actions_index_ < robot_actions_.size() && running_executeRobotActions_){
            if(robot_actions_[robot_actions_index_].isFinished()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "process: %d", robot_actions_index_);
                robot_actions_index_++;
                if(robot_actions_index_ < robot_actions_.size() && robot_actions_index_ <= robot_actions_index_to_){
                    executeRobotAction(robot_actions_index_);
                } else {
                    running_executeRobotActions_ = false;
                    
                }
            }
        }
    };
    void abort() {
        if(robot_actions_index_ < robot_actions_.size()){
            robot_actions_[robot_actions_index_].abort();
        }
    };
    void clear() {
        for(auto &robot_action : robot_actions_){
            robot_action.abort();
        }
        robot_actions_.clear();
        running_executeRobotActions_ = false;
        robot_actions_index_ = 0;
        robot_actions_index_to_ = 0;
    }
    std::string getStatus() {
        if(robot_actions_index_ < robot_actions_.size()){
            // std::cout << running_robot_action_->getStatus() << std::endl;
            return robot_actions_[robot_actions_index_].getStatus();
        }
        return "done";
    };
    bool isFinishedAction(size_t index) {
        if(index < robot_actions_.size()){
            return robot_actions_[index].isFinished();
        }
        return true;
    };
    bool isFinished() {
        return !running_executeRobotActions_;
    };
    size_t getRobotActionsIndex() {
        return robot_actions_index_;
    };
    size_t getRobotActionNum() {
        return robot_actions_.size();
    };
    std::vector<Line> getCurrentPath() {
        if(robot_actions_index_ < robot_actions_.size())if(robot_actions_[robot_actions_index_].path_.path.size() >= 2){
            std::vector<Line> lines;
            for (size_t i = 0; i < robot_actions_[robot_actions_index_].path_.path.size() - 1; i++) {
                Line l;
                l.from = {(float)robot_actions_[robot_actions_index_].path_.path[i].x, (float)robot_actions_[robot_actions_index_].path_.path[i].y};
                l.to = {(float)robot_actions_[robot_actions_index_].path_.path[i + 1].x, (float)robot_actions_[robot_actions_index_].path_.path[i + 1].y};
                lines.push_back(l);
            }
            return lines;
        }
        return std::vector<Line>();
    };
    
private:
    rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client_;
    rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client_;
    rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bonbori_pub_;
    // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client_;
    std::vector<RobotAction> robot_actions_;
    // RobotAction *running_robot_action_;
    size_t robot_actions_index_;
    bool running_executeRobotActions_ = false;
    size_t robot_actions_index_to_;

    void daiza_feedback_callback(rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::SharedPtr goal_handle, const std::shared_ptr<const mecha_control::action::DaizaCmd::Feedback> feedback){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Daiza feedback callback");
        for(auto &robot_action : robot_actions_){
            robot_action.get_daiza_feedback_callback() (goal_handle, feedback);
        }
    }
    void daiza_result_callback(const rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::WrappedResult & result){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Daiza result callback");
        for(auto &robot_action : robot_actions_){
            robot_action.get_daiza_result_callback() (result);
        }
    }
    void hina_feedback_callback(rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::SharedPtr goal_handle, const std::shared_ptr<const mecha_control::action::HinaCmd::Feedback> feedback){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hina feedback callback");
        for(auto &robot_action : robot_actions_){
            robot_action.get_hina_feedback_callback() (goal_handle, feedback);
        }
    }
    void hina_result_callback(const rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::WrappedResult & result){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hina result callback");
        for(auto &robot_action : robot_actions_){
            robot_action.get_hina_result_callback() (result);
        }
    }
    void move_path_feedback_callback(rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::SharedPtr goal_handle, const std::shared_ptr<const pure_pursuit::action::PathAndFeedback::Feedback> feedback){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move path feedback callback");
        auto daiza_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions();
        auto hina_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions();
        daiza_cmd_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::daiza_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        daiza_cmd_send_goal_options.result_callback = std::bind(&RobotActionsManager::daiza_result_callback, this, std::placeholders::_1);
        for(auto &robot_action : robot_actions_){
            robot_action.get_move_path_feedback_callback() (goal_handle, feedback, daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
        }
    }
    void move_path_result_callback(const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult & result){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move path result callback");
        auto daiza_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions();
        auto hina_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions();
        daiza_cmd_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::daiza_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        daiza_cmd_send_goal_options.result_callback = std::bind(&RobotActionsManager::daiza_result_callback, this, std::placeholders::_1);
        for(auto &robot_action : robot_actions_){
            robot_action.get_move_path_result_callback() (result, daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
        }
    }
};

class PathCallers
{
public:
    PathCallers(rclcpp::Client<pure_pursuit::srv::GetPath>::SharedPtr get_path_client) : get_path_client_(get_path_client) {
    };
    void RequestPath(uint8_t path_name) {
        if (!get_path_client_->wait_for_service(std::chrono::seconds(1))) {
            return;
        }
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting path : %d", path_name);
        request_num_++;
        auto request = std::make_shared<pure_pursuit::srv::GetPath::Request>();
        request->path_number = path_name;
        auto result = get_path_client_->async_send_request(request, std::bind(&PathCallers::get_path_result_callback, this, std::placeholders::_1));
    };
    void RequestPaths(std::vector<uint8_t> path_names) {
        for(auto path_name : path_names){
            RequestPath(path_name);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    };
    bool isFinished() {
        if(request_num_ == ok_num_){
            return true;
        }
        return false;
    };
    std::vector<pure_pursuit::msg::Path2DWithParams> getPaths() {
        return paths_;
    };
    pure_pursuit::msg::Path2DWithParams getPath(uint8_t path_name) {
        for(size_t i = 0; i < path_number_.size(); i++){
            if(path_number_[i] == path_name){
                return paths_[i];
            }
        }
        return pure_pursuit::msg::Path2DWithParams();
    };
    RobotAction::MechActionPath GetMechActionPath(uint8_t path_name) {
        for(size_t i = 0; i < path_number_.size(); i++){
            if(path_number_[i] == path_name){
                return mech_action_paths_[i];
            }
        }
        return RobotAction::MechActionPath();
    }
    int getOkNum() {
        return ok_num_;
    };
    int getRequestNum() {
        return request_num_;
    };
private:
    rclcpp::Client<pure_pursuit::srv::GetPath>::SharedPtr get_path_client_;
    std::vector<uint8_t> path_number_;
    int request_num_ = 0;
    int ok_num_ = 0;
    std::vector<pure_pursuit::msg::Path2DWithParams> paths_;
    std::vector<RobotAction::MechActionPath> mech_action_paths_;
    void get_path_result_callback(rclcpp::Client<pure_pursuit::srv::GetPath>::SharedFutureWithRequest future){
        auto result = future.get();
        path_number_.push_back(result.first.get()->path_number);
        paths_.push_back(result.second.get()->path);
        RobotAction::MechActionPath mech_action_path{};
        for(size_t i = 0; i < result.second.get()->indices.size(); i++){
            mech_action_path.mech_action_indices.push_back(result.second.get()->indices[i]);
            mecha_control::msg::MechAction mech_action_daiza{};
            mech_action_daiza.type = mecha_control::msg::MechAction::DAIZA;
            mech_action_daiza.daiza.command = result.second.get()->daiza_commands[i];
            mech_action_path.mech_actions_path.push_back(mech_action_daiza);

            mech_action_path.mech_action_indices.push_back(result.second.get()->indices[i]);
            mecha_control::msg::MechAction mech_action_hina{};
            mech_action_hina.type = mecha_control::msg::MechAction::HINA;
            mech_action_hina.hina.command = result.second.get()->hina_commands[i];
            mech_action_path.mech_actions_path.push_back(mech_action_hina);

            mech_action_path.mech_action_indices.push_back(result.second.get()->indices[i]);
            mecha_control::msg::MechAction mech_action_bonbori{};
            mech_action_bonbori.type = mecha_control::msg::MechAction::BONBORI;
            mech_action_bonbori.bonbori_enable = result.second.get()->bonbori_commands[i];
            mech_action_path.mech_actions_path.push_back(mech_action_bonbori);
        }
        mech_action_paths_.push_back(mech_action_path);
        ok_num_++;
    }
};

