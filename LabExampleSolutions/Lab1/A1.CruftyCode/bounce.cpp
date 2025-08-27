#include <iostream>

class Ball
{
    public:
        void bounce() 
        {   
            std::cout << "The ball bounces!" << std::endl; 
            ++bounceCount;
        }
        int bounceCount;
};

class Basket
{
    public:
        void init() 
        {
            hasBall = false;
        }
        void placeBall()
        {
            if (hasBall == false) 
            {
                std::cout << "Placing the ball in the basket..." << std::endl;
                ball.bounce();
                hasBall = true;
            }
            else
                std::cout << "The basket already has a ball." << std::endl;
        }

        bool hasBall;
        Ball ball;
};

int main() 
{
    Basket basket;

    basket.placeBall();
    basket.placeBall();

    return 0;
}

