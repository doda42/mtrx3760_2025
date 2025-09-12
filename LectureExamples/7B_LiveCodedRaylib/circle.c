// Refactor this ChatGPT-generated program to be object-oriented
// Hint: write a wrapper class for the C-based raylib
//
// Install the raylib library with:
// sudo add-apt-repository ppa:texus/raylib
// sudo apt update
// sudo apt install libraylib5-dev
//
// build with
// gcc circle.c -lraylib


#include "raylib.h"
#include <stdlib.h>   // for rand()


int main(void) {
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "Bouncing Circle with Gravity (C)");
    SetTargetFPS(60);

    // Circle properties
    float x = 400, y = 300;
    float radius = 50;
    float dx = 4.0f;      // horizontal velocity
    float dy = 0.0f;      // vertical velocity
    const float gravity = 0.5f;   // acceleration due to gravity
    const float damping = 0.98f;   // energy loss on bounce

    while (!WindowShouldClose()) {
        // Apply gravity
        dy += gravity;

        // Update position
        x += dx;
        y += dy;

        // Bounce off floor
        if (y + radius > screenHeight) {
            y = screenHeight - radius;  // reposition at floor
            
            // Reverse velocity and add a small random variation
            float randomFactor = ((rand() % 21)) / 25.0f;
            dy *= -damping;
            dy -= randomFactor;
        }

        // Bounce off ceiling
        if (y - radius < 0) {
            y = radius;
            dy *= -damping;
        }

        // Bounce off walls
        if (x - radius < 0 || x + radius > screenWidth) dx *= -1;

        // Draw
        BeginDrawing();
        ClearBackground(BLACK);
        DrawCircle((int)x, (int)y, radius, RED);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}

