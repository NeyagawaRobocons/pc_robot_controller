#include <raylib.h>
#include <complex>

class RobotChassis4 {
    private:
        Vector2 _pos;
        float _scale;
        float _scale_vec;
        float _scale_wheel_vel;
        float _wheel_dimention_radius;
        Color _color;
        Vector2 _robot_lines[5];
        Vector2 _robot_wheels_vec[4];
        float _x_vel;
        float _y_vel;
        float _w_vel;
        float _wheels_vel[4];
    public:
        RobotChassis4(Vector2 pos, float scale, float scale_vec, float scale_wheel_vel, float wheel_dimention_radius, Color color){
            _pos = pos;
            _scale = scale;
            _scale_vec = scale_vec;
            _scale_wheel_vel = scale_wheel_vel;
            _wheel_dimention_radius = wheel_dimention_radius;
            _color = color;
            _robot_lines[0] = {pos.x + std::sqrt(2) * wheel_dimention_radius * scale / 2, pos.y + std::sqrt(2) * wheel_dimention_radius * scale / 2};
            _robot_lines[1] = {pos.x - std::sqrt(2) * wheel_dimention_radius * scale / 2, pos.y + std::sqrt(2) * wheel_dimention_radius * scale / 2};
            _robot_lines[2] = {pos.x - std::sqrt(2) * wheel_dimention_radius * scale / 2, pos.y - std::sqrt(2) * wheel_dimention_radius * scale / 2};
            _robot_lines[3] = {pos.x + std::sqrt(2) * wheel_dimention_radius * scale / 2, pos.y - std::sqrt(2) * wheel_dimention_radius * scale / 2};
            _robot_lines[4] = _robot_lines[0];
            _robot_wheels_vec[0] = {- std::sqrt(2) / 2, std::sqrt(2) / 2};
            _robot_wheels_vec[1] = {- std::sqrt(2) / 2, - std::sqrt(2) / 2};
            _robot_wheels_vec[2] = {std::sqrt(2) / 2, - std::sqrt(2) / 2};
            _robot_wheels_vec[3] = {std::sqrt(2) / 2, std::sqrt(2) / 2};
        }
        void draw(){
            DrawSplineLinear(_robot_lines, 5, 4.0, GRAY);
            DrawCircleSector(_pos, _wheel_dimention_radius * _scale /2, 90, _w_vel * 180 / PI + 90, 50, SKYBLUE);
            DrawSplineSegmentLinear({_pos.x, _pos.y}, {_pos.x + _scale_vec * _scale * _x_vel, _pos.y + _scale_vec * _scale * _y_vel}, 4.0, RED);
            for(size_t i = 0; i < 4; i++){
                DrawSplineSegmentLinear({_robot_lines[i].x, _robot_lines[i].y}, {_robot_lines[i].x + _scale_wheel_vel * _scale * _robot_wheels_vec[i].x * _wheels_vel[i], _robot_lines[i].y + _scale_wheel_vel * _scale * _robot_wheels_vec[i].y * _wheels_vel[i]}, 4.0, PINK);
            }
        }
        void draw_y_inv(){
            DrawSplineLinear(_robot_lines, 5, 4.0, GRAY);
            DrawCircleSector(_pos, _wheel_dimention_radius * _scale /2, -90, - _w_vel * 180 / PI - 90, 50, SKYBLUE);
            DrawCircleSector(_pos, _wheel_dimention_radius * _scale /2 - 8, -90, - _w_vel * 180 / PI - 90, 50, RAYWHITE);
            DrawSplineSegmentLinear({_pos.x, _pos.y}, {_pos.x + _scale_vec * _scale * _x_vel, _pos.y - _scale_vec * _scale * _y_vel}, 4.0, RED);
            for(size_t i = 0; i < 4; i++){
                Vector2 v_origin = _robot_lines[3 - i];
                DrawSplineSegmentLinear({v_origin.x, v_origin.y}, {v_origin.x + _scale_wheel_vel * _scale * _robot_wheels_vec[i].x * _wheels_vel[i], v_origin.y - _scale_wheel_vel * _scale * _robot_wheels_vec[i].y * _wheels_vel[i]}, 4.0, PINK);
            }
        }
        void set_vel(float x, float y, float w){
            _x_vel = x;
            _y_vel = y;
            _w_vel = w;
        }
        void set_wheels_vel(float w[4]){
            for (size_t i = 0; i < 4; i++)
            {
                _wheels_vel[i] = w[i];
            }
        }
        void set_pos(Vector2 pos){
            _pos = pos;
            _robot_lines[0] = {pos.x + std::sqrt(2) * _wheel_dimention_radius * _scale / 2, pos.y + std::sqrt(2) * _wheel_dimention_radius * _scale / 2};
            _robot_lines[1] = {pos.x - std::sqrt(2) * _wheel_dimention_radius * _scale / 2, pos.y + std::sqrt(2) * _wheel_dimention_radius * _scale / 2};
            _robot_lines[2] = {pos.x - std::sqrt(2) * _wheel_dimention_radius * _scale / 2, pos.y - std::sqrt(2) * _wheel_dimention_radius * _scale / 2};
            _robot_lines[3] = {pos.x + std::sqrt(2) * _wheel_dimention_radius * _scale / 2, pos.y - std::sqrt(2) * _wheel_dimention_radius * _scale / 2};
            _robot_lines[4] = _robot_lines[0];
        }
};
