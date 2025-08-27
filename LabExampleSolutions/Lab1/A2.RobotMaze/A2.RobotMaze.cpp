// Simple robot maze simulation
// This version is incomplete
// fill in the "stub" functions to make a robot that follows the right wall
// to eventually reach the 'X'
// Hint: you may need to add a member variable to the robot so it remembers what it's doing
// Do not change the parts of the implementation that are already complete and working, only complete the stub functions

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

const int WIDTH = 20;
const int HEIGHT = 10;
enum Direction { UP, RIGHT, DOWN, LEFT };

class Maze;
class Robot;

//-----------------------------------------------------------------------------------
struct Pose 
{
    int x;
    int y;
    Direction dir;
};

//-----------------------------------------------------------------------------------
class Robot 
{
    public:
        void init( Maze* maze );
    
        Pose getPose();
        
        Pose findPoseFwd();
        Pose findPoseRight();

        bool senseRightFree();
        bool senseFwdFree();        
                
        void moveFwd();
        void turnRight();
        void turnLeft();   
        
        void takeOneAction();

    private:
        Pose pose;
        Maze* pMaze;
        bool justTurned;
};


//-----------------------------------------------------------------------------------
class Maze 
{
    public:
        void init();
        bool isFree( Pose pose );
        bool isDone( Pose pose );
        char getCellDrawing( int y, int x );

    private:
        std::vector<std::string> layout;
};

//-----------------------------------------------------------------------------------
class Renderer 
{
    public:
        void init( Maze* maze, Robot* robot );
        void clear();
        void draw();
        
    private:
        Maze* pMaze;
        Robot* pRobot;
};

//-----------------------------------------------------------------------------------
class Simulation 
{
    public:
        void init();
        void run();
    
    private:
        Robot robot;
        Maze maze;
        Renderer render;
};


//-----------------------------------------------------------------------------------
int main() 
{
    Simulation theSim;
    theSim.init();
    theSim.run();

    return 0;
}

//-----------------------------------------------------------------------------------
void Renderer::init(Maze* maze, Robot* robot) 
{
    pMaze = maze; 
    pRobot = robot;
}
void Renderer::clear() 
{
    std::cout << "\033[2J\033[H";
}
void Renderer::draw() 
{
    clear();
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            if (y == pRobot->getPose().y && x == pRobot->getPose().x)
                switch( pRobot->getPose().dir ){
                    case UP: std::cout << '^'; break;
                    case DOWN: std::cout << 'v'; break;
                    case LEFT: std::cout << '<'; break;
                    case RIGHT: std::cout << '>'; break;
                }
            else
                std::cout << pMaze->getCellDrawing(y, x);
        }
        std::cout << "\n";
    }
    std::cout << std::flush;
}

//-----------------------------------------------------------------------------------
void Maze::init() 
{
    layout = {
        "********************",
        "*     *            *",
        "***** ***********  *",
        "*        *X     *  *",
        "*   *********   *  *",
        "*               *  *",
        "* ***********      *",
        "*         *     *  *",
        "*         *     *  *",
        "********************"
    };
}
char Maze::getCellDrawing(int y, int x) 
{
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
        return '*';
    return layout[y][x];
}
bool Maze::isFree( Pose pose ) 
{
    return layout[pose.y][pose.x] == ' ';
}
bool Maze::isDone( Pose pose ) 
{
    return layout[pose.y][pose.x] == 'X';
}

//-----------------------------------------------------------------------------------
void Simulation::init()
{
    maze.init();
    robot.init(&maze);
    render.init(&maze, &robot);
}
void Simulation::run()
{
    while (true) {
        render.draw();
        robot.takeOneAction();
        if( maze.isDone( robot.findPoseFwd() ) )
        {
            std::cout << "DONE!" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // delay
    }
}

//-----------------------------------------------------------------------------------
void Robot::init( Maze* maze ) 
{
    pose.x = 1;
    pose.y = 1;
    pose.dir = RIGHT;
    pMaze = maze;
    justTurned = false;
}
Pose Robot::getPose() 
{ 
    return pose; 
}
Pose Robot::findPoseFwd()
{
    Pose newPose = pose;
    switch( pose.dir )
    {
        case UP: --newPose.y; break;
        case DOWN: ++newPose.y; break;        
        case LEFT: --newPose.x; break;
        case RIGHT: ++newPose.x; break;
    }
    return newPose;
}
Pose Robot::findPoseRight()
{
    Pose newPose = pose;
    switch( pose.dir )
    {
        case UP: ++newPose.x; break;
        case DOWN: --newPose.x; break;        
        case LEFT: --newPose.y; break;
        case RIGHT: ++newPose.y; break;
    }
    return newPose;
}

void Robot::moveFwd()
{
    if( senseFwdFree() )
        pose = findPoseFwd();   
}
void Robot::turnRight()
{
    pose.dir = static_cast<Direction>((pose.dir + 1) % 4);
}
void Robot::turnLeft()
{
    pose.dir = static_cast<Direction>((pose.dir + 3) % 4);
}


bool Robot::senseFwdFree()
{
    bool result = true;
    std::cout << "STUB implementation, always sensing forward as free" << std::endl;
    return result;
}
bool Robot::senseRightFree()
{
    bool result = true;
    std::cout << "STUB implementation, always sensing to the right as free" << std::endl;
    return result;
}

void Robot::takeOneAction() 
{
    moveFwd();
    std::cout << "STUB implementation, always moving forward" << std::endl;
}

