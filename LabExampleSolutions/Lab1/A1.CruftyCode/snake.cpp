#include <iostream>
#include <cstdlib>
#include <ctime>
#include <unistd.h> // for usleep


#define WIDTH 40
#define HEIGHT 20
#define DELAY 100000 // in microseconds

class Snake {
public:
    int x, y;

    Snake();

    void draw();
    void moveRandom();
};

// ---------- main function ----------
int main() {
    srand(time(0));

    Snake snake;

    while (true) {
        snake.draw();
        snake.moveRandom();

        // Wrap around screen edges
        if (snake.x < 0) snake.x = WIDTH - 1;
        if (snake.x >= WIDTH) snake.x = 0;
        if (snake.y < 0) snake.y = HEIGHT - 1;
        if (snake.y >= HEIGHT) snake.y = 0;

        usleep(DELAY);
    }

    return 0;
}

// ---------- function implementations ----------

Snake::Snake() {
    x = WIDTH / 2;
    y = HEIGHT / 2;
}

void Snake::draw() {
    // Draw to the screen
    
    std::cout << "\033[2J\033[1;1H"; // clear

    for (int row = 0; row < HEIGHT; row++) {
        for (int col = 0; col < WIDTH; col++) {
            if (row == y && col == x)
                std::cout << '*';
            else
                std::cout << ' ';
        }
        std::cout << '\n';
    }

    std::cout << std::flush;
}

void Snake::moveRandom() {
    // Now apply random movement
    int dir = rand() % 4;

    if (dir == 0) x++;       // right
    if (dir == 1) x--;       // left
    if (dir == 2) y++;       // down
    if (dir == 3) y--;       // up
}
