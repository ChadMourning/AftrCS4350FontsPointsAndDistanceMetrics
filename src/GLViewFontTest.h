#pragma once

#include "GLView.h"

class NvFlexLibrary;
class NvFlexSolverDesc;
class NvFlexBuffer;
class NvFlexSolver;

namespace Aftr
{
   class Camera;
   class MGLPointSetShaderAccelerated;




/**
   \class GLViewFontTest
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewFontTest : public GLView
{
public:
   static GLViewFontTest* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewFontTest();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void createFontTestWayPoints();
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );

protected:
   GLViewFontTest( const std::vector< std::string >& args );
   virtual void onCreate();   

   MGLPointSetShaderAccelerated* mgl = nullptr;

   NvFlexLibrary* library = nullptr;
   NvFlexSolverDesc* desc;

   NvFlexSolver* solver = nullptr;

   NvFlexBuffer* particleBuffer = nullptr;
   NvFlexBuffer* velocityBuffer = nullptr;
   NvFlexBuffer* phaseBuffer = nullptr;
   NvFlexBuffer* activeBuffer = nullptr;

};

/** \} */

} //namespace Aftr
