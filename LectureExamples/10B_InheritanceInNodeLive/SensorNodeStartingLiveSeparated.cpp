// A start on the node design for the fire sensor problem
// See FireSensorProblemStatement.txt
//
// In this example we'll use a stand-in CNode that knows how to "publish" by
// printing to the screen as a stand-in for ROS 2. This allows for lightweight
// development during the early design stage for this project.

#include "HardwareEmulator.h"

#include <iostream>
#include <string>

//---Pretend-Node--------------------------------------------------------------
// A minimalist class to stand in for ROS in small examples
class CNode
{
  public:
    void Publish( std::string Message );
};

//-----------------------------------------------------------------------------
class History
{
};

//---Fwd decl------------------------------------------------------------------
class FireDetector;

//-----------------------------------------------------------------------------
class FireDetectorNode: public CNode
{
  public:
    FireDetectorNode();
    ~FireDetectorNode();
    FireDetector* GetDetector() { return mpDetector; }
  private:
    FireDetector* mpDetector;
};

//-----------------------------------------------------------------------------
class FireDetector: public ISensorCallable
{
  public:
  private:
    History MyHistory;
};

//-----------------------------------------------------------------------------
class FireDetectorPN: public FireDetector
{
  public:
    void Callback( int Value )
    {
      std::cout << "PN got value " << Value << std::endl;
//      Publish( "PN Received" );
    }
};

//-----------------------------------------------------------------------------
class FireDetectorCam: public FireDetector
{
  public:
    void Callback( int Value )
    {
      std::cout << "Cam got value " << Value << std::endl;
//      Publish( "Cam Received" );      
    }
};

//-----------------------------------------------------------------------------
int main()
{
 
  FireDetectorNode MyDetector;
  if( MyDetector.GetDetector() != NULL )
  {
    CHardwareEmulator MyEmulator( *MyDetector.GetDetector() );
    // wait for a bit to let some events happen
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  return 0;
}


//-----------------------------------------------------------------------------
void CNode::Publish( std::string Message )
{
  std::cout << Message << std::endl;
}

FireDetectorNode::FireDetectorNode()
{
  int Choice = 0;
  std::cout << "What type? [0,1] " << std::endl;
  std::cin >> Choice;

  switch( Choice )
  {
    case 0:
      mpDetector = new FireDetectorCam;
    break;
    case 1:
      mpDetector = new FireDetectorPN;    
    break;
    default:
      std::cout << "oops" << std::endl;
  }
}


FireDetectorNode::~FireDetectorNode() 
{ 
  if( mpDetector) 
  {
    delete mpDetector; 
  }
}
