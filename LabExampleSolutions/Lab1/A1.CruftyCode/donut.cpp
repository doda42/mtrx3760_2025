#include <iostream>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>

int main() {
    float A = 0, B = 0;
    const int width = 80;
    const int height = 22;
    float z[1760];
    char b[1760];

    while (true) {
        memset(b, 32, 1760);
        memset(z, 0, 7040);
        for (float j = 0; j < 6.28; j += 0.07) {
            for (float i = 0; i < 6.28; i += 0.02) {
                float c = sin(i), d = cos(j), e = sin(A),
                      f = sin(j), g = cos(A), h = d + 2,
                      D = 1 / (c * h * e + f * g + 5),
                      l = cos(i), m = cos(B), n = sin(B),
                      t = c * h * g - f * e;

                int x = int(width / 2 + 30 * D * (l * h * m - t * n));
                int y = int(height / 2 + 15 * D * (l * h * n + t * m));
                int o = x + width * y;
                int N = int(8 * ((f * e - c * d * g) * m - c * d * e - f * g - l * d * n));
                if (22 > y && y > 0 && x > 0 && width > x && D > z[o]) {
                    z[o] = D;
                    b[o] = ".,-~:;=!*#$@"[std::max(0, N)];
                }
            }
        }

        std::cout << "\033[H"; // move cursor to top
        for (int k = 0; k < 1760; k++)
            std::cout << (k % width ? b[k] : '\n');
        A += 0.04;
        B += 0.02;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    return 0;
}

