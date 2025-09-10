// Example solution for Hello, Shapes
// This is meant to illustrate the correct structure of the program
// It doesn't necessarily reflect all best coding practices, 
//    e.g. commenting
//    e.g. implementing functions inline in declaration

#include <iostream>

struct Position
{
  int x;
  int y;
};

class Shape
{
  public:
    Shape() : position{0, 0} { std::cout << "Shape CTor" << std::endl; }
    virtual ~Shape() { std::cout << "Shape DTor" << std::endl; }
    virtual void draw() = 0;
  protected:
    Position position;
};

class Circle: public Shape
{
  public:
    Circle() { std::cout << "Circle CTor" << std::endl; }
    ~Circle() { std::cout << "Circle DTor" << std::endl; }    
    virtual void draw();
};
class Square: public Shape
{
  public:
    Square() : angle(0.0) { std::cout << "Square CTor" << std::endl; }
    ~Square() { std::cout << "Square DTor" << std::endl; }    
    virtual void draw();
    double angle;
};
class Triangle: public Shape
{
  public:
    Triangle() : angle(0.0) { std::cout << "Triangle CTor" << std::endl; }
    ~Triangle() { std::cout << "Triangle DTor" << std::endl; }    
    virtual void draw();
    double angle;
};

class House: public Shape
{
  public:
    House() : angle(0.0) { std::cout << "House CTor" << std::endl; }
    ~House() { std::cout << "House DTor" << std::endl; }    
    virtual void draw();
    double angle;
  private:
    Square square;
    Triangle triangle;
};

class Duplex: public Shape
{
  public:
    Duplex() : angle(0.0) { std::cout << "Duplex CTor" << std::endl; }
    ~Duplex() { std::cout << "Duplex DTor" << std::endl; }    
    virtual void draw();
    double angle;
  private:
    House house[2];
};


int main()
{
  const int NumShapes = 6;
 	Shape* allShapes[NumShapes];


  // Requested order: Square, Circle, Duplex, Triangle, House, Circle
  // 1 0 4 2 3 0

  std::cout << "Enter shape order, list 6 numbers in range 0..4" << std::endl;
	for( int i=0; i<NumShapes; ++i )
	{
		int ShapeType;
		std::cin >> ShapeType;
		
		std::cout << "Creating shape type " << ShapeType << std::endl;
		Shape* pNewShape = NULL;
		switch( ShapeType )
		{
			case 0: pNewShape = new Circle; break;
			case 1: pNewShape = new Square; break;
			case 2: pNewShape = new Triangle; break;						
			case 3: pNewShape = new House; break;
			case 4: pNewShape = new Duplex; break;
		};
		
		allShapes[i] = pNewShape;
	}
  	

	for( int i=0; i<NumShapes; ++i )
	{
		allShapes[i]->draw();
	}

	for( int i=0; i<NumShapes; ++i )
	{
		delete allShapes[i];
	}


}

void Circle::draw()
{
  std::cout << "Drawing circle at " << position.x << ", " << position.y << std::endl;
}
void Square::draw()
{
  std::cout << "Drawing square at " << position.x << ", " << position.y << " orientation " << angle << std::endl;
}
void Triangle::draw()
{
  std::cout << "Drawing triangle at " << position.x << ", " << position.y << " orientation " << angle << std::endl;
}

void House::draw()
{
  std::cout << "Drawing house at " << position.x << ", " << position.y << " orientation " << angle << std::endl;
  square.draw();
  triangle.draw();
  std::cout << "Done drawing house" << std::endl;
}
void Duplex::draw()
{
  std::cout << "Drawing duplex at " << position.x << ", " << position.y << " orientation " << angle << std::endl;
  house[0].draw();
  house[1].draw();
  std::cout << "Done drawing duplex" << std::endl;
}
