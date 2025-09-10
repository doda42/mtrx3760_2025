// This is the solution to the lab 2 shapes problem as we started it in lecture
// Time got a bit tight, so I'm also including a more complete solution in the same folder
// The more complete solution does not have an OrientedShape. It would be interesting to 
// complete this program or refactor that one to use OrientedShape.

#include <iostream>

struct Position
{
  int x;
  int y;
};

class Shape  // abstract base class
{
  public:
    Shape()
    {
      position.x = 0;
      position.y = 0;      
    }
    virtual void draw() = 0; // pure virtual function
  protected:
    Position position;
};

class OrientedShape: public Shape // abstract base class
{
  public:
    OrientedShape()
    {
      orientation_deg = 0;
    }
  protected:
    double orientation_deg;
};

class Circle: public Shape
{
  public:
    void draw() { std::cout << "Circle draw at " << position.x << ", " << position.y << std::endl; }
};

class Square: public OrientedShape
{
  public:
    void draw() { std::cout << "Square draw at " << position.x << ", " << position.y << " oriented: " << orientation_deg << std::endl; }
};

class Triangle: public OrientedShape
{
  public:
    void draw() { std::cout << "Triangle draw at " << position.x << ", " << position.y << " oriented: " << orientation_deg << std::endl; }
};



int main()
{
  const int NumShapes = 6;
  Shape* myShapes[NumShapes];
  
  for( int i=0; i<NumShapes; ++i )
  {
    int shapeSel;
    std::cout << "Which shape?";
    std::cin >> shapeSel;
    
    switch( shapeSel )
    {
      case 0: myShapes[i] = new Circle;
      break;
      case 1: myShapes[i] = new Square;
      break;
      default: std::cout << "undefined shape" << std::endl;
    };
  }

  for( int i=0; i<NumShapes; ++i )
  {
    myShapes[i]->draw();
  }

    
  for( int i=0; i<NumShapes; ++i )
  {
    delete myShapes[i];
  }
    
}


//---
