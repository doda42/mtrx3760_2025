#include <iostream>

using namespace std;

class Wheel {
    public:
        void rotate();
    private:
        int turns;
};

class Car {
    public:
        void drive();
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
    cout << "Car is driving..." << endl;
    frontLeft.rotate();
    frontRight.rotate();
    backLeft.rotate();
    backRight.rotate();
}

void Wheel::rotate() {
    cout << "Wheel is rotating" << endl;
    this->turns++;
}
