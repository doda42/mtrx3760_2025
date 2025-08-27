#include <iostream>

// Wheels for a car, can rotate
class Wheel {
    public:
        void rotate();          // A wheel tracks its turns, this rotates 
    private:
        int turns;              // Tracking wheel turns
};

// Car knows how to drive, has four wheels
class Car {
    public:
        void drive();           // drives the car, manipulates wheels
        
        // The four wheels of the car
        Wheel frontLeft;
        Wheel frontRight;
        Wheel backLeft;
        Wheel backRight;
};


int main() {
    Car myCar;
    myCar.drive();

    return 0;
}


void Car::drive() {
    std::cout << "Car is driving..." << std::endl;
    frontLeft.rotate();
    frontRight.rotate();
    backLeft.rotate();
    backRight.rotate();
}

void Wheel::rotate() {
    std::cout << "Wheel is rotating" << std::endl;
    this->turns++;
}
