#pragma once
#include "raylib.h"
#include "raymath.h"
#include "robot_action.hpp"
#include "button.hpp"

class RobotActionList
{
public:
    RobotActionList(Vector2 origin, std::string name, RobotActionsManager *manager): start_button({origin.x + 200, origin.y}, 100, 70, GRAY, GREEN, false, 0.5){
        this->origin = origin;
        this->name = name;
        this->manager = manager;
    }
    void add_action(RobotAction action, std::string action_name){
        actions.push_back(action);
        action_names.push_back(action_name);
    }
    void append_action_to_manager(){
        for (auto action : actions)
        {
            indices.push_back(manager->addRobotAction(action));
        }
    }
    void move(Vector2 origin){
        this->origin = origin;
        start_button.move({origin.x + 200, origin.y});
    }
    void process(Vector2 mouse){
        start_button.process(mouse);
        if(start_button.is_pressed() && manager->isFinished()){
            manager->executeRobotActions(indices[0], indices[indices.size() - 1]);
        }
    }
    void draw(){
        DrawText(name.c_str(), origin.x, origin.y + 5, 20, BLACK);
        std::stringstream ss;
        std::string status = manager->getStatus();
        ss << "Status: " << status;
        DrawText(ss.str().c_str(), origin.x, origin.y + 80, 20, BLACK);
        DrawText("Actions:", origin.x, origin.y + 105, 20, BLACK);
        for (size_t i = 0; i < action_names.size(); i++)
        {
            DrawText(action_names[i].c_str(), origin.x, origin.y + 135 + 25 * i, 20, manager->isFinishedAction(i) ? BLACK : GRAY);
        }
        start_button.draw();
        DrawRectangleRoundedLines({origin.x - 5, origin.y - 5, 300.0 + 5.0, 135.0 + action_names.size() *25 + 5}, 0.1, 50, 4.0, GRAY);
    }
private:
    RobotActionsManager *manager;
    std::vector<RobotAction> actions;
    std::vector<std::string> action_names;
    std::vector<size_t> indices;
    std::string name;
    Vector2 origin;
    push_button start_button;
};