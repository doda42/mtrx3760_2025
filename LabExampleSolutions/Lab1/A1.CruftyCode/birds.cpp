#include <iostream>
#include <cstdlib>
#include <ctime>
#include <unistd.h> // for usleep

const int WIDTH = 40;
const int HEIGHT = 20;
const int NUM_mBirds = 10;
const int DELAY = 100000; // microseconds between frames

class Bird {
public:
    Bird();
    void move_random();
    int get_x();
    int GetY();
    
private:
    int x; 
    int y;
};

class CFlock {
public:
    void MoveAll();
    void draw();
    
private:
    Bird mBirds[NUM_mBirds];
};

// ------------------------- main -------------------------
int main() {
    std::srand(std::time(0));
    CFlock CFlock;
    
    while (true) {
        CFlock.MoveAll();
        CFlock.draw();
        usleep(DELAY);
    }
    
    return 0;
}

// -------------------- Bird implementation --------------------
Bird::Bird() {
    x = std::rand() % WIDTH;
    y = std::rand() % HEIGHT;
}

void Bird::move_random() {
    int dx = (std::rand() % 3) - 1; // -1, 0, or 1
    int dy = (std::rand() % 3) - 1;
    
    x = (x + dx + WIDTH) % WIDTH;
    y = (y + dy + HEIGHT) % HEIGHT;
}

int Bird::get_x()  {
    return x;
}

int Bird::GetY()  {
    return y;
}

// -------------------- CFlock implementation --------------------
void CFlock::MoveAll() {
    for (int i = 0; i < NUM_mBirds; ++i) {
        mBirds[i].move_random();
    }
}

void CFlock::draw()  {
    // Clear screen
    std::cout << "\033[2J\033[1;1H";
    
    // Empty screen buffer
    char screen[HEIGHT][WIDTH];
    for (int row = 0; row < HEIGHT; ++row) {
        for (int col = 0; col < WIDTH; ++col) {
            screen[row][col] = ' ';
        }
    }
    
    // Mark bird positions
    for (int i = 0; i < NUM_mBirds; ++i) {
        int x = mBirds[i].get_x();
        int y = mBirds[i].GetY();
        screen[y][x] = '*';
    }
    
    // Print screen
    for (int row = 0; row < HEIGHT; ++row) {
        for (int col = 0; col < WIDTH; ++col) {
            std::cout << screen[row][col];
        }
        std::cout << '\n';
    }
    
    std::cout << std::flush;
}

