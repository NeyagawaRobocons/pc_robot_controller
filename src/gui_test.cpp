#include <iostream>
#include <raylib.h>

#include "slider.hpp"
#include "vector_calc.hpp"
#include "button.hpp" 
#include "label.hpp"

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");

    SetTargetFPS(60);
    Vector2 start = {100, 100};
    Vector2 end = {400, 100};
    slider s(start, end, 10, RED);
    push_button b({100, 200}, 200, 50, GRAY, BLUE);
    label l("label", {100, 300}, 20, RED);
    while (!WindowShouldClose())
    {
        auto mouse = GetMousePosition();
        if (b.process(mouse)){
            l.set_text("button pressed");
        }
        s.process(mouse);
        BeginDrawing();
        ClearBackground(RAYWHITE);
        s.draw();
        b.draw();
        l.draw();
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
