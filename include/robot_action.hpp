#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "pure_pursuit/msg/path2_d_with_speed.hpp"
#include "mecha_control/msg/mech_action.hpp"
#include "pure_pursuit/action/path_and_feedback.hpp"
#include "mecha_control/action/daiza_cmd.hpp"
#include "mecha_control/action/hina_cmd.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client_;
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
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client
    ) {
        action_type_ = ActionType::MOVE_PATH_WITH_MECH_ACTION;
        path_ = path;
        mech_action_path_ = mech_action_path;
        move_path_action_client_ = move_path_action_client;
        daiza_cmd_action_client_ = daiza_cmd_action_client;
        hina_cmd_action_client_ = hina_cmd_action_client;
        bonbori_srv_client_ = bonbori_srv_client;
    };
    RobotAction(mecha_control::msg::MechAction mech_action, 
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client,
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client,
        bool non_blocking=false
    ) {
        action_type_ = non_blocking ? ActionType::MECH_ACTION_NON_BLOCKING : ActionType::MECH_ACTION;
        mech_action_ = mech_action;
        daiza_cmd_action_client_ = daiza_cmd_action_client;
        hina_cmd_action_client_ = hina_cmd_action_client;
        bonbori_srv_client_ = bonbori_srv_client;
    };
    RobotAction(ActionType action_type, pure_pursuit::msg::Path2DWithParams path, MechActionPath mech_action_path, mecha_control::msg::MechAction mech_action,
        rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client,
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client
    ) {
        action_type_ = action_type;
        path_ = path;
        mech_action_path_ = mech_action_path;
        mech_action_ = mech_action;
        move_path_action_client_ = move_path_action_client;
        daiza_cmd_action_client_ = daiza_cmd_action_client;
        hina_cmd_action_client_ = hina_cmd_action_client;
        bonbori_srv_client_ = bonbori_srv_client;
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
    std::function<void(const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult &)> get_move_path_result_callback() {
        if(action_type_ == ActionType::MOVE_PATH){
            return std::bind(&RobotAction::executeMovePath_result_callback, this, std::placeholders::_1);
        } else if(action_type_ == ActionType::MOVE_PATH_WITH_MECH_ACTION){
            return std::bind(&RobotAction::executeMovePathWithMechAction_result_callback, this, std::placeholders::_1);
        } else {
            return [](const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult &){};
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
        auto move_path_goal_handle_future = move_path_action_client_->async_send_goal(move_path_goal_msg, move_path_send_goal_options);
        move_path_goal_uuid_ = move_path_goal_handle_future.get()->get_goal_id();
        return move_path_goal_uuid_;
    }
    void executeMovePathWithMechAction_feedback_callback(rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::SharedPtr goal_handle,
        const std::shared_ptr<const pure_pursuit::action::PathAndFeedback::Feedback> feedback,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options
    ){
        if(goal_handle->get_goal_id() == move_path_goal_uuid_){
            std::stringstream ss;
            path_current_index_ = feedback->current_index;
            ss << "Feedback: " << path_current_index_;
            status_ = ss.str();
            if(mech_action_path_.mech_action_indices.size() > 0){
                if(path_current_index_ >= mech_action_path_.mech_action_indices[0]){
                    mech_action_ = mech_action_path_.mech_actions_path[0];
                    executeMechAction(daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
                    mech_action_path_.mech_action_indices.erase(mech_action_path_.mech_action_indices.begin());
                    mech_action_path_.mech_actions_path.erase(mech_action_path_.mech_actions_path.begin());
                }
            }
        }
    }
    void executeMovePathWithMechAction_result_callback(const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult & result){
        if(result.goal_id == move_path_goal_uuid_){
            std::stringstream ss;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    ss << "Goal";
                    ss << " with " << result.result->final_index;
                    while(mech_action_path_.mech_action_indices.size() > 0){
                        if(path_current_index_ >= mech_action_path_.mech_action_indices[0]){
                            mech_action_ = mech_action_path_.mech_actions_path[0];
                            executeMechAction(rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions(), rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions());
                            mech_action_path_.mech_action_indices.erase(mech_action_path_.mech_action_indices.begin());
                            mech_action_path_.mech_actions_path.erase(mech_action_path_.mech_actions_path.begin());
                        }
                        if(mech_action_path_.mech_action_indices.size() == 0){
                            break;
                        }
                        if(mech_action_path_.mech_action_indices[0] > result.result->final_index){
                            ss << " but cannot execute " << mech_action_path_.mech_action_indices.size() <<" mech action";
                            break;
                        }
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
    rclcpp_action::GoalUUID executeMechAction(rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions daiza_cmd_send_goal_options,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions hina_cmd_send_goal_options
    ) {
        rclcpp_action::GoalUUID goal_uuid;
        if (!daiza_cmd_action_client_->wait_for_action_server(std::chrono::seconds(1)) || !hina_cmd_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            status_ = "Action server not available";
            return goal_uuid;
        }
        if(mech_action_.type == mecha_control::msg::MechAction::DAIZA){
            auto daiza_cmd_goal_msg = mecha_control::action::DaizaCmd::Goal();
            daiza_cmd_goal_msg.command = mech_action_.daiza.command;
            auto daiza_cmd_goal_handle_future = daiza_cmd_action_client_->async_send_goal(daiza_cmd_goal_msg, daiza_cmd_send_goal_options);
            this->daiza_cmd_goal_uuid_ = daiza_cmd_goal_handle_future.get()->get_goal_id();
            goal_uuid = daiza_cmd_goal_uuid_;
        } else if(mech_action_.type == mecha_control::msg::MechAction::HINA){
            auto hina_cmd_goal_msg = mecha_control::action::HinaCmd::Goal();
            hina_cmd_goal_msg.command = mech_action_.hina.command;
            auto hina_cmd_goal_handle_future = hina_cmd_action_client_->async_send_goal(hina_cmd_goal_msg, hina_cmd_send_goal_options);
            this->hina_cmd_goal_uuid_ = hina_cmd_goal_handle_future.get()->get_goal_id();
            goal_uuid = hina_cmd_goal_uuid_;
        } else if(mech_action_.type == mecha_control::msg::MechAction::BONBORI){
            auto bonbori_srv_request = std_srvs::srv::SetBool::Request::SharedPtr();
            bonbori_srv_request->data = mech_action_.bonbori_enable;
            auto bonbori_srv_future = bonbori_srv_client_->async_send_request(bonbori_srv_request, std::bind(&RobotAction::bonbori_result_callback, this, std::placeholders::_1));
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
    void bonbori_result_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
        std::stringstream ss;
        switch (future.get()->success) {
            case true:
                ss << "Bonbori :" << mech_action_.bonbori_enable;
                break;
            case false:
                ss << "Bonbori faled";
                break;
        }
        status_ = ss.str();
        finished_ = true;
    }

};

class RobotActionsManager
{
public:
    RobotActionsManager(rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client,
        rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client,
        rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client,
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client
    ) : move_path_action_client_(move_path_action_client), daiza_cmd_action_client_(daiza_cmd_action_client), hina_cmd_action_client_(hina_cmd_action_client), bonbori_srv_client_(bonbori_srv_client) {
    };
    size_t addRobotAction(RobotAction robot_action) {
        robot_actions_.push_back(robot_action);
        return robot_actions_.size() - 1;
    };
    void executeRobotAction(size_t index) {
        if(index < robot_actions_.size()){
            auto robot_action = robot_actions_[index];
            auto move_path_send_goal_options = rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SendGoalOptions();
            auto daiza_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions();
            auto hina_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions();
            daiza_cmd_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::daiza_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            daiza_cmd_send_goal_options.result_callback = std::bind(&RobotActionsManager::daiza_result_callback, this, std::placeholders::_1);
            hina_cmd_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::hina_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            hina_cmd_send_goal_options.result_callback = std::bind(&RobotActionsManager::hina_result_callback, this, std::placeholders::_1);
            move_path_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::move_path_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            move_path_send_goal_options.result_callback = std::bind(&RobotActionsManager::move_path_result_callback, this, std::placeholders::_1);
            robot_action.execute(move_path_send_goal_options, daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
            robot_actions_index_ = index;
            robot_actions_[robot_actions_index_].move_path_goal_uuid_ = robot_action.move_path_goal_uuid_;
            robot_actions_[robot_actions_index_].daiza_cmd_goal_uuid_ = robot_action.daiza_cmd_goal_uuid_;
            robot_actions_[robot_actions_index_].hina_cmd_goal_uuid_ = robot_action.hina_cmd_goal_uuid_;
            robot_actions_[robot_actions_index_].bonbori_srv_request_id_ = robot_action.bonbori_srv_request_id_;
            robot_actions_[robot_actions_index_].finished_ = robot_action.finished_;
            robot_actions_[robot_actions_index_].status_ = robot_action.status_;
            running_robot_action_ = &robot_action;
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
                robot_actions_index_++;
                if(robot_actions_index_ < robot_actions_.size() && robot_actions_index_ <= robot_actions_index_to_){
                    executeRobotAction(robot_actions_index_);
                } else {
                    running_executeRobotActions_ = false;
                    
                }
            }
        }
    };
    std::string getStatus() {
        if(running_robot_action_ != nullptr && robot_actions_index_ < robot_actions_.size()){
            // std::cout << running_robot_action_->getStatus() << std::endl;
            return robot_actions_[robot_actions_index_].getStatus();
        }
        return "done";
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
private:
    rclcpp_action::Client<pure_pursuit::action::PathAndFeedback>::SharedPtr move_path_action_client_;
    rclcpp_action::Client<mecha_control::action::DaizaCmd>::SharedPtr daiza_cmd_action_client_;
    rclcpp_action::Client<mecha_control::action::HinaCmd>::SharedPtr hina_cmd_action_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bonbori_srv_client_;
    std::vector<RobotAction> robot_actions_;
    RobotAction *running_robot_action_;
    size_t robot_actions_index_;
    bool running_executeRobotActions_ = false;
    size_t robot_actions_index_to_;

    void daiza_feedback_callback(rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::SharedPtr goal_handle, const std::shared_ptr<const mecha_control::action::DaizaCmd::Feedback> feedback){
        for(auto &robot_action : robot_actions_){
            robot_action.get_daiza_feedback_callback() (goal_handle, feedback);
        }
    }
    void daiza_result_callback(const rclcpp_action::ClientGoalHandle<mecha_control::action::DaizaCmd>::WrappedResult & result){
        for(auto &robot_action : robot_actions_){
            robot_action.get_daiza_result_callback() (result);
        }
    }
    void hina_feedback_callback(rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::SharedPtr goal_handle, const std::shared_ptr<const mecha_control::action::HinaCmd::Feedback> feedback){
        for(auto &robot_action : robot_actions_){
            robot_action.get_hina_feedback_callback() (goal_handle, feedback);
        }
    }
    void hina_result_callback(const rclcpp_action::ClientGoalHandle<mecha_control::action::HinaCmd>::WrappedResult & result){
        for(auto &robot_action : robot_actions_){
            robot_action.get_hina_result_callback() (result);
        }
    }
    void move_path_feedback_callback(rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::SharedPtr goal_handle, const std::shared_ptr<const pure_pursuit::action::PathAndFeedback::Feedback> feedback){
        auto daiza_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::DaizaCmd>::SendGoalOptions();
        auto hina_cmd_send_goal_options = rclcpp_action::Client<mecha_control::action::HinaCmd>::SendGoalOptions();
        daiza_cmd_send_goal_options.feedback_callback = std::bind(&RobotActionsManager::daiza_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        daiza_cmd_send_goal_options.result_callback = std::bind(&RobotActionsManager::daiza_result_callback, this, std::placeholders::_1);
        for(auto &robot_action : robot_actions_){
            robot_action.get_move_path_feedback_callback() (goal_handle, feedback, daiza_cmd_send_goal_options, hina_cmd_send_goal_options);
        }
    }
    void move_path_result_callback(const rclcpp_action::ClientGoalHandle<pure_pursuit::action::PathAndFeedback>::WrappedResult & result){
        for(auto &robot_action : robot_actions_){
            robot_action.get_move_path_result_callback() (result);
        }
    }
};
