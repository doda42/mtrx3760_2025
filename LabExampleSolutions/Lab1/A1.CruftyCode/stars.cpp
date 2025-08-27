#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>

const int SCREEN_WIDTH = 80;
const int SCREEN_HEIGHT = 24;
const int STAR_COUNT = 100;

class Star {
    public:
        Star() {
            reset();
        }

        void update() {
            y += velocity;
            if (y >= SCREEN_HEIGHT) {
                reset();
            }
        }

        void draw() {
            moveCursor((int)y, x);
            std::cout << symbol;
        }

    private:
        int x;
        float y;
        float velocity;
        char symbol;

        void reset();
        void moveCursor(int row, int col) {
            std::cout << "\033[" << row + 1 << ";" << col + 1 << "H";
        }
};

class StarField {
    public:
        StarField(int count) {
            for (int i = 0; i < count; ++i)
                stars.push_back(Star());
        }

        void update() {
            for (auto& s : stars)
                s.update();
        }

        void draw() {
            for (auto& s : stars)
                s.draw();
        }
        
    private:
        std::vector<Star> stars;
};

class Screen {
    public:
        void clear() {
            std::cout << "\033[2J";
        }

        void hideCursor() {
            std::cout << "\033[?25l";
        }

        void showCursor() {
            std::cout << "\033[?25h";
        }

        void screenflush() {
            std::cout << std::flush;
        }
};

int main() {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    Screen screen;
    screen.hideCursor();
    StarField field(STAR_COUNT);

    while (true) {
        screen.clear();
        field.update();
        field.draw();
        screen.screenflush();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    screen.showCursor(); // Unreachable, but left for symmetry
    return 0;
}

void Star::reset() {
x = rand() % SCREEN_WIDTH;
y = 0;
velocity = 0.1f + static_cast<float>(rand()) / RAND_MAX * 0.5f;
symbol = (rand() % 3 == 0) ? '*' : '.';
}

