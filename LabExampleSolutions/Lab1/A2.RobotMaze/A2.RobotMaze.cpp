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
// A robot that knows how to wall-follow to solve a simple maze
// Interacts with the Maze class to sense andavoid going through walls
class Robot 
{
    public:
        void init( Maze* maze ); // init to starting pose, associate with maze
    
        Pose getPose();          // returns the robot's pose
        
        Pose findPoseFwd();      // returns pose one in front of robot
        Pose findPoseRight();    // returns pose one to right of robot

        bool senseRightFree();   // detects if the spot to the right is free
        bool senseFwdFree();     // detects if the spot directly ahead is free
                
        void moveFwd();          // moves the robot forward
        void turnRight();        // turns the robot right
        void turnLeft();         // turns the robot left
        
        void takeOneAction();    // uses sensors to decide on and take a single action from the list above

    private:
        Pose pose;
        Maze* pMaze;
        bool justTurned;
};


//-----------------------------------------------------------------------------------
// Simple maze with a hard-coded layout
class Maze 
{
    public:
        void init();                // initialise the maze, hard-coded
        bool isFree( Pose pose );   // check if the location Pose is free
        bool isDone( Pose pose );   // check if Pose is the finishing 'X'
        char getCellDrawing( int y, int x );    // gets the symbol to draw for a particular cell location

    private:
        std::vector<std::string> layout;
};

//-----------------------------------------------------------------------------------
// Renderer knows how to render a maze with a robot in it
class Renderer 
{
    public:
        // Init the renderer, keeps associations with robot and maze
        void init( Maze* maze, Robot* robot );
        void clear();   // clear the display
        void draw();    // draw the display
        
    private:
        Maze* pMaze;
        Robot* pRobot;
};

//-----------------------------------------------------------------------------------
// The top-level simulation contains a robot, maze, and renderer
class Simulation 
{
    public:
        void init();    // initialise the simulation
        void run();     // run the simulation
    
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
    Pose checkPose = findPoseFwd();
    result = pMaze->isFree( checkPose );
    return result;
}

bool Robot::senseRightFree()
{
    bool result = true;
    Pose checkPose = findPoseRight();
    result = pMaze->isFree( checkPose );
    return result;
}

void Robot::takeOneAction() 
{
    if( !justTurned && senseRightFree() )
    {
        turnRight();
        justTurned = true;
        std::cout << "Turn right" << std::endl;
    } 
    else if( senseFwdFree() )
    {
        moveFwd();
        justTurned = false;
        std::cout << "Fwd" << std::endl;
    } 
    else 
    {
        turnLeft();
        justTurned = true;
        std::cout << "Turn left" << std::endl;
    }                
}

