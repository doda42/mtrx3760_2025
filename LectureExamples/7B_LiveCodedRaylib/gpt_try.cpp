#include "raylib.h"
#include <cstdlib>   // for rand()
#include <ctime>     // for seeding rand

// -------------------------------------------------------
// Wrapper around raylib window and drawing
// -------------------------------------------------------
class Renderer {
public:
    Renderer(int width, int height, const char* title, int fps = 60) {
        InitWindow(width, height, title);
        SetTargetFPS(fps);
        screenWidth = width;
        screenHeight = height;
    }

    ~Renderer() {
        CloseWindow();
    }

    bool ShouldClose() const {
        return WindowShouldClose();
    }

    void BeginFrame() const {
        BeginDrawing();
        ClearBackground(BLACK);
    }

    void EndFrame() const {
        EndDrawing();
    }

    int GetWidth() const { return screenWidth; }
    int GetHeight() const { return screenHeight; }

private:
    int screenWidth;
    int screenHeight;
};

// -------------------------------------------------------
// Ball class with physics
// -------------------------------------------------------
class Ball {
public:
    Ball(float x, float y, float r, float dx, float dy)
        : posX(x), posY(y), radius(r), velX(dx), velY(dy) {}

    void Update(const Renderer& renderer) {
        // Apply gravity
        velY += gravity;

        // Update position
        posX += velX;
        posY += velY;

        int screenW = renderer.GetWidth();
        int screenH = renderer.GetHeight();

        // Bounce off floor
        if (posY + radius > screenH) {
            posY = screenH - radius;
            velY *= -damping;
            velY -= RandomFactor();
        }

        // Bounce off ceiling
        if (posY - radius < 0) {
            posY = radius;
            velY *= -damping;
        }

        // Bounce off walls
        if (posX - radius < 0 || posX + radius > screenW) {
            velX *= -1;
        }
    }

    void Draw() const {
        DrawCircle((int)posX, (int)posY, radius, RED);
    }

private:
    float posX, posY;
    float radius;
    float velX, velY;

    static constexpr float gravity = 0.5f;
    static constexpr float damping = 0.98f;

    float RandomFactor() const {
        return (rand() % 21) / 25.0f; // random between 0 and 0.8
    }
};

// -------------------------------------------------------
// Main program
// -------------------------------------------------------
int main() {
    srand((unsigned)time(nullptr)); // seed randomness

    Renderer renderer(800, 600, "Bouncing Circle with Gravity (C++)");

    Ball ball(400, 300, 50, 4.0f, 0.0f);

    while (!renderer.ShouldClose()) {
        ball.Update(renderer);

        renderer.BeginFrame();
        ball.Draw();
        renderer.EndFrame();
    }

    return 0;
}

