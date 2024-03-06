#pragma once
#include "raylib.h"
#include "raymath.h"
#include "robot_action.hpp"
#include "button.hpp"
#include "label.hpp"

class RobotActionList
{
public:
    RobotActionList(Vector2 origin, std::string name, RobotActionsManager *manager): start_button({origin.x + 150, origin.y}, 150, 70, GREEN, LIME, false, 0.5), start_button_label("Start", {origin.x + 160, origin.y +25}, 20, DARKGRAY){
        this->origin = origin;
        this->name = name;
        this->manager = manager;
    }
    void add_action(RobotAction action, std::string action_name){
        actions.push_back(action);
        action_names.push_back(action_name);
    }
    void append_action_to_manager_all(){
        manager->clear();
        for (auto action : actions)
        {
            indices.push_back(manager->addRobotAction(action));
        }
        index_from_ = 0;
        index_to_ = actions.size() - 1;
    }
    // void append_action_to_manager(size_t index){
    //     indices.push_back(manager->addRobotAction(actions[index]));
    // }
    void clear(){
        actions.clear();
        action_names.clear();
        indices.clear();
    }
    void clear_manager(){
        manager->clear();
    }
    void clear_all(){
        clear();
        clear_manager();
    }
    void change_action_to_manager(size_t index_from, size_t index_to){
        manager->clear();
        indices.clear();
        for (size_t i = index_from; i <= index_to; i++)
        {
            indices.push_back(manager->addRobotAction(actions[i]));
        }
        index_from_ = index_from;
        index_to_ = index_to;
    }
    void move(Vector2 origin){
        this->origin = origin;
        start_button.move({origin.x + 150, origin.y});
        start_button_label.move({origin.x + 160, origin.y + 25});
    }
    void process(Vector2 mouse, bool push_start = false){
        start_button.process(mouse);
        if(push_start){
            start_button.push();
        }
        if(start_button.is_pressed() && manager->isFinished()){
            std::cout << "Start button pressed" << std::endl;
            manager->executeRobotActions(0, index_to_ - index_from_ + 1);
        }
    }
    void set_name(std::string name){
        this->name = name;
    }
    void draw(){
        DrawText(name.c_str(), origin.x, origin.y + 5, 20, BLACK);
        std::stringstream ss;
        std::string status = manager->getStatus();
        ss << "Status: " << status;
        DrawText(ss.str().c_str(), origin.x, origin.y + 80, 20, BLACK);
        ss.clear();
        ss << "Actions: " << index_from_ << " - " << index_to_;
        DrawText(ss.str().c_str(), origin.x, origin.y + 105, 20, BLACK);
        for (size_t i = index_from_; i <= index_to_; i++)
        {
            DrawText(action_names[i].c_str(), origin.x, origin.y + 135 + 25 * (i - index_from_), 20, manager->isFinishedAction(i - index_from_) ? DARKGRAY : PINK);
        }
        start_button.draw();
        start_button_label.draw();
        DrawRectangleRoundedLines({origin.x - 5, origin.y - 5, 300.0 + 10, 135 + 25 + (index_to_ - index_from_) *25 + 5}, 0.1, 50, 4.0, GRAY);
    }
    size_t size(){
        return actions.size();
    }
private:
    RobotActionsManager *manager;
    size_t index_from_;
    size_t index_to_;
    std::vector<RobotAction> actions;
    std::vector<std::string> action_names;
    std::vector<size_t> indices;
    std::string name;
    Vector2 origin;
    push_button start_button;
    label start_button_label;
    
};