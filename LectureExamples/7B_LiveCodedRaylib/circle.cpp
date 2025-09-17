// Refactored
//
// Install the raylib library with:
//
// sudo add-apt-repository ppa:texus/raylib
// sudo apt update
// sudo apt install libraylib5-dev
//
// build with
// g++ circle.cpp -lraylib


#include "raylib.h"

#include <stdlib.h>   // for rand()
#include <iostream>


//-----------------------------------------------------------------------------
struct Vec2D
{
    float x;
    float y;
};

//-----------------------------------------------------------------------------
class CRender
{
    public:
        //---Ctor/Dtor---
        CRender();
        
        //---Util---
        bool WindowShouldClose();
        void CloseWindow();
            
        //---Drawing---
        void BeginDrawing();
        void EndDrawing();    

        void DrawCircle( Vec2D aPosition, int aRadius, Color aColor );
        
        //---Consts---
        const int mScreenWidth;
        const int mScreenHeight;
};

//-----------------------------------------------------------------------------
class CBall
{
    public:
        //---Ctor/Dtor---    
        CBall( CRender& arRender );
        
        //---Physics simulation---
        void Update();
        
        //---Rendering---
        void Draw();
       
    private:
        //---
        Vec2D mPosition;
        Vec2D mVelocity;
        float mRadius;
        
        //---Consts---
        const float mDamping;   // energy loss on bounce

        // TODO: mGravity really isn't part of the ball, put somewhere better        
        const float mGravity;   // acceleration due to mGravity
        
        //---Renderer---
        CRender& mrRender;        
};

//-----------------------------------------------------------------------------
int main()
{
    CRender Render;
    CBall Ball( Render );
    
    //---The main loop---
    while( !Render.WindowShouldClose() ) 
    {
        Render.BeginDrawing();
        
        Ball.Update();

        Ball.Draw();
                        
        Render.EndDrawing();
    }
    
    //---Cleanup---
    Render.CloseWindow();
    
    return 0;
}


//-----------------------------------------------------------------------------
CBall::CBall( CRender& arRender )
    :   mPosition( {400.0f + (100.0f * (rand()/RAND_MAX + 0.5f)), 300.0f} ),
        mVelocity( {4.0f, 0.0f} ),
        mRadius( 50.0f ),
        mDamping( 0.98f ),
        mGravity( 0.5f ),
        mrRender( arRender )
{
    
}

void CBall::Update()
{
    mVelocity.y += mGravity;

    // Update position // TODO: use or program a vector that knows how to mPosition += mVelocity;
    mPosition.x += mVelocity.x;
    mPosition.y += mVelocity.y;

    // Bounce off floor
    if( mPosition.y + mRadius > mrRender.mScreenHeight ) 
    {
        mPosition.y = mrRender.mScreenHeight - mRadius;  // reposition at floor
        
        // Reverse velocity and add a small random variation
        float randomOffset = ((rand() % 21)) / 25.0f;
        mVelocity.y *= -mDamping;
        mVelocity.y -= randomOffset;
    }

    // Bounce off ceiling
    if (mPosition.y - mRadius < 0) 
    {
        mPosition.y = mRadius;
        mVelocity.y *= -mDamping;
    }

    // Bounce off walls
    if (mPosition.x - mRadius < 0 || mPosition.x + mRadius > mrRender.mScreenWidth) 
    {
        mVelocity.x *= -1;
    }
}


void CBall::Draw()
{
    mrRender.DrawCircle( mPosition, mRadius, RED );
}

//-----------------------------------------------------------------------------
CRender::CRender()
    :
        mScreenWidth( 800 ),
        mScreenHeight( 600 )
{
    InitWindow( mScreenWidth, mScreenHeight, "Bouncing Circle with Gravity (C++)");
    SetTargetFPS(60);
}

bool CRender::WindowShouldClose()
{
    bool Result = ::WindowShouldClose();
    return Result;
}


void CRender::CloseWindow()
{
    ::CloseWindow();
}

void CRender::BeginDrawing()
{
    ::BeginDrawing();
    ::ClearBackground( BLACK );
}
void CRender::EndDrawing()
{
    ::EndDrawing();
}

void CRender::DrawCircle( Vec2D aPosition, int aRadius, Color aColor )
{
    ::DrawCircle( aPosition.x, aPosition.y, aRadius, aColor );
}

