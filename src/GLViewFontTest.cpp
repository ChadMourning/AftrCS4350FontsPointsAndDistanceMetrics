#include "GLViewFontTest.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"
#include "AftrUtilities.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "AftrGLRendererBase.h"

//If we want to use way points, we need to include this.
#include "FontTestWayPoints.h"
#include "FTGLString.h"
#include "WOFTGLString.h"
//#include "WOGUILabel.h"
#include "MGLFTGLString.h"
//#include "WOGUITextBox.h"
#include "WOCameraSink.h"
#include "AftrFrameBufferObject.h"
#include "MGLPointSetShaderAccelerated.h"
#include "ModelMeshDataShared.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"

#include "MGLFrustum.h"
#include "boost/asio.hpp"

//#include "NvFlex.h"
#include "MGLPointCloud.h"
#include "GLSLShaderPointTesselatorBillboard.h"
#include "WOCameraSink.h"

using namespace Aftr;

GLViewFontTest* GLViewFontTest::New( const std::vector< std::string >& args )
{
   GLViewFontTest* glv = new GLViewFontTest( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewFontTest::GLViewFontTest( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewFontTest::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewFontTest::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewFontTest::onCreate()
{
   //GLViewFontTest::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   this->setNumPhysicsStepsPerRender( 1 ); //pause physics engine on start up; will remain paused till set to 1

   boost::asio::io_service io_service;
   boost::asio::ip::tcp::resolver resolver(io_service);
}


GLViewFontTest::~GLViewFontTest()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewFontTest::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
}


void GLViewFontTest::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewFontTest::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewFontTest::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewFontTest::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewFontTest::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_1 )
   {
      ModelMeshRenderData* r = mgl->getModelDataShared()->getModelMeshes().at(0)->getMeshDataShared()->getModelMeshRenderData(MESH_SHADING_TYPE::mstNONE, GL_POINTS);
      r->mapVBODataToClientMemory();
      for (size_t k = 0; k < 10000; k++)
      {
         aftrColor4ub color(rand() % 255, rand() % 255, rand() % 255, 255);
         r->setColorArrayAtIndexInVertexList((unsigned int)k, &color);
      }
      r->unMapVBODataFromClientMemory();

   } 
   if (key.keysym.sym == SDLK_2)//euclidian
   {
      ModelMeshRenderData* r = mgl->getModelDataShared()->getModelMeshes().at(0)->getMeshDataShared()->getModelMeshRenderData(MESH_SHADING_TYPE::mstNONE, GL_POINTS);
      r->mapVBODataToClientMemory();
      for (size_t k = 0; k < 10000; k++)
      {
         float radius = 30;
         Vector v(rand() % 255, rand() % 255, rand() % 255);
         v.normalize();
         v *= radius;

         GLubyte* ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * k + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = (rand() % 2 ? -1 : 1) *  v.x + 100;
         ((GLfloat*)ptr)[1] = (rand() % 2 ? -1 : 1) * v.y + 100;
         ((GLfloat*)ptr)[2] = (rand() % 2 ? -1 : 1) * v.z + 100;

         aftrColor4ub color(rand() % 255, rand() % 255, rand() % 255, 255);
         r->setColorArrayAtIndexInVertexList((unsigned int)k, &color);
      }
      r->unMapVBODataFromClientMemory();

   }
   if (key.keysym.sym == SDLK_3)//manhattan
   {
      ModelMeshRenderData* r = mgl->getModelDataShared()->getModelMeshes().at(0)->getMeshDataShared()->getModelMeshRenderData(MESH_SHADING_TYPE::mstNONE, GL_POINTS);
      r->mapVBODataToClientMemory();
      for (size_t k = 0; k < 10000; k++)
      {
         int radius = 30;
         int x = rand() % (radius * 1000);
         int y = rand() % (radius * 1000 - x);
         int z = rand() % (radius * 1000 - x - y);

         float x1 = (rand() % 2 ? -1 : 1) * x / 1000.0f;
         float y1 = (rand() % 2 ? -1 : 1) * y / 1000.0f;
         float z1 = (rand() % 2 ? -1 : 1) * z / 1000.0f;

         GLubyte* ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * k + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = x1 + 100;
         ((GLfloat*)ptr)[1] = y1 + 100;
         ((GLfloat*)ptr)[2] = z1 + 100;

         aftrColor4ub color(rand() % 255, rand() % 255, rand() % 255, 255);
         r->setColorArrayAtIndexInVertexList((unsigned int)k, &color);
      }
      r->unMapVBODataFromClientMemory();

   }
   if (key.keysym.sym == SDLK_4)//chebyshev
   {
      ModelMeshRenderData* r = mgl->getModelDataShared()->getModelMeshes().at(0)->getMeshDataShared()->getModelMeshRenderData(MESH_SHADING_TYPE::mstNONE, GL_POINTS);
      r->mapVBODataToClientMemory();
      for (size_t k = 0; k < 10000-6; k+=6)
      {
         int radius = 30;
         Vector v(rand() % 255, rand() % 255, rand() % 255);
         v.normalize();

         if (v.x > v.y && v.x > v.z)
         {
            v/=v.x;
         }
         else if (v.y > v.x && v.y > v.z)
         {
            v /= v.y;
         }
         else if (v.z > v.x && v.z > v.y)
         {
            v /= v.z;
         }

         v *= radius;
         aftrColor4ub color(rand() % 255, rand() % 255, rand() % 255, 255);

         GLubyte* ptr;
         int i = 0;
         ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * (k + i) + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = (rand() % 2 ? -1 : 1) * v.x + 100;
         ((GLfloat*)ptr)[1] = (rand() % 2 ? -1 : 1) * v.y + 100;
         ((GLfloat*)ptr)[2] = (rand() % 2 ? -1 : 1) * v.z + 100;
         r->setColorArrayAtIndexInVertexList((unsigned int)k+i, &color);
         i++;
         ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * (k + i) + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = (rand() % 2 ? -1 : 1) * v.x + 100;
         ((GLfloat*)ptr)[1] = (rand() % 2 ? -1 : 1) * v.z + 100;
         ((GLfloat*)ptr)[2] = (rand() % 2 ? -1 : 1) * v.y + 100;
         r->setColorArrayAtIndexInVertexList((unsigned int)k + i, &color);
         i++;
         ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * (k + i) + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = (rand() % 2 ? -1 : 1) * v.y + 100;
         ((GLfloat*)ptr)[1] = (rand() % 2 ? -1 : 1) * v.x + 100;
         ((GLfloat*)ptr)[2] = (rand() % 2 ? -1 : 1) * v.z + 100;
         r->setColorArrayAtIndexInVertexList((unsigned int)k + i, &color);
         i++;
         ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * (k + i) + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = (rand() % 2 ? -1 : 1) * v.y + 100;
         ((GLfloat*)ptr)[1] = (rand() % 2 ? -1 : 1) * v.z + 100;
         ((GLfloat*)ptr)[2] = (rand() % 2 ? -1 : 1) * v.x + 100;
         r->setColorArrayAtIndexInVertexList((unsigned int)k + i, &color);
         i++;
         ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * (k + i) + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = (rand() % 2 ? -1 : 1) * v.z + 100;
         ((GLfloat*)ptr)[1] = (rand() % 2 ? -1 : 1) * v.x + 100;
         ((GLfloat*)ptr)[2] = (rand() % 2 ? -1 : 1) * v.y + 100;
         r->setColorArrayAtIndexInVertexList((unsigned int)k + i, &color);
         i++;
         ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * (k + i) + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = (rand() % 2 ? -1 : 1) * v.z + 100;
         ((GLfloat*)ptr)[1] = (rand() % 2 ? -1 : 1) * v.y + 100;
         ((GLfloat*)ptr)[2] = (rand() % 2 ? -1 : 1) * v.x + 100;
         r->setColorArrayAtIndexInVertexList((unsigned int)k + i, &color);
            
            
      
      }
      r->unMapVBODataFromClientMemory();

   }
   else if (key.keysym.sym == SDLK_2)
   {
      /*
      float* particles = (float*)NvFlexMap(particleBuffer, eNvFlexMapWait);
      float* velocities = (float*)NvFlexMap(velocityBuffer, eNvFlexMapWait);
      int* phases = (int*)NvFlexMap(phaseBuffer, eNvFlexMapWait);
      
      //update rendering bodies
      ModelMeshRenderData* r = mgl->getModelDataShared()->getModelMeshes().at(0)->getMeshDataShared()->getModelMeshRenderData(MESH_SHADING_TYPE::mstNONE, GL_POINTS);
      r->mapVBODataToClientMemory();
      for (int i = 0; i < 1000; i++)
      {
        // std::cout << particles[i * 4 + 0] << " " << particles[i * 4 + 1] << " " << particles[i * 4 + 2] << std::endl;
         GLubyte* ptr = ((GLubyte*)r->getGlMapBuffer()) + r->getVtxStride() * i + r->getVertsOffset();
         ((GLfloat*)ptr)[0] = particles[i*4+0];
         ((GLfloat*)ptr)[1] = particles[i*4+1];
         ((GLfloat*)ptr)[2] = particles[i*4+2];
      }
      r->unMapVBODataFromClientMemory();

      /*
      // unmap buffers
      NvFlexUnmap(particleBuffer);
      NvFlexUnmap(velocityBuffer); 
      NvFlexUnmap(phaseBuffer);

      // write to device (async)
      NvFlexSetParticles(solver, particleBuffer, nullptr);
      NvFlexSetVelocities(solver, velocityBuffer, nullptr);
      NvFlexSetPhases(solver, phaseBuffer, nullptr);

      // tick
      float dt = .1;
      NvFlexUpdateSolver(solver, dt, 1, false);

      // read back (async)
      NvFlexGetParticles(solver, particleBuffer, nullptr);
      NvFlexGetVelocities(solver, velocityBuffer, nullptr);
      NvFlexGetPhases(solver, phaseBuffer, nullptr);
      */

   }
   else if (key.keysym.sym == SDLK_3)
   {
      // map buffers for reading / writing
      /*
      float* particles = (float*)NvFlexMap(particleBuffer, eNvFlexMapWait);
      float* velocities = (float*)NvFlexMap(velocityBuffer, eNvFlexMapWait);
      int* phases = (int*)NvFlexMap(phaseBuffer, eNvFlexMapWait);

      // spawn (user method)
      for (int i = 0; i < 1000; i++)
      {
         particles[i * 4 + 0] = rand() % 100;//x
         particles[i * 4 + 1] = rand() % 100;//y
         particles[i * 4 + 2] = 100;//z
         particles[i * 4 + 3] = 1000;//1/mass
         velocities[i * 3 + 0] = (rand() % 1000) / 250.0;
         velocities[i * 3 + 1] = (rand() % 1000) / 500.0;
         velocities[i * 3 + 2] = (rand() % 1000) / 500.0;
         phases[i] = NvFlexMakePhase(1, 0);// eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid |
      }
      
      // unmap buffers
      NvFlexUnmap(particleBuffer);
      NvFlexUnmap(velocityBuffer);
      NvFlexUnmap(phaseBuffer);

      NvFlexSetParticles(solver, particleBuffer, nullptr);
      NvFlexSetVelocities(solver, velocityBuffer, nullptr);
      NvFlexSetPhases(solver, phaseBuffer, nullptr);
      NvFlexSetActiveCount(solver, 1000);
      */
   }
}


void GLViewFontTest::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void GLViewFontTest::loadMap()
{
   Vector v(1, 0, 0);
   v = v.rotate(Vector(0, 0, 1), Aftr::PI / 2);
   v = v.rotate(Vector(1, 0, 0), Aftr::PI / 2);

   /*
   library = NvFlexInit();
   desc = new NvFlexSolverDesc();
   desc->maxParticles = 1000;
   desc->maxNeighborsPerParticle = 32;
   desc->maxDiffuseParticles = 0;
   solver = NvFlexCreateSolver(library,desc);

   NvFlexParams params;
   NvFlexGetParams(solver, &params);
   params.gravity[0] = 0;
   params.gravity[1] = 0;
   params.gravity[2] = -1;
   params.numIterations = 1;
   params.radius = 1;
   NvFlexSetParams(solver, &params);

    particleBuffer = NvFlexAllocBuffer(library, 1000, sizeof(float) * 4, eNvFlexBufferHost);
    velocityBuffer = NvFlexAllocBuffer(library, 1000, sizeof(float) * 3, eNvFlexBufferHost);
    phaseBuffer = NvFlexAllocBuffer(library, 1000, sizeof(int), eNvFlexBufferHost);
    */

   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE( 1000.0 );
   ManagerOpenGLState::GL_NEAR_PLANE( 0.1f );
   ManagerOpenGLState::enableFrustumCulling( false );
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg" );

   float ga = 0.1f; //Global Ambient Light level for this module
   ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
   WOLight* light = WOLight::New();
   light->isDirectionalLight( true );
   light->setPosition( Vector( 0, 0, 100 ) );
   //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
   //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
   light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
   light->setLabel( "Light" );
   worldLst->push_back( light );

   //Create the SkyBox
   WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
   wo->setPosition( Vector( 0,0,0 ) );
   wo->setLabel( "Sky Box" );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   worldLst->push_back( wo );

   ////Create the infinite grass plane (the floor)
   wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   wo->setPosition( Vector( 0, 0, 0 ) );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   wo->upon_async_model_loaded([wo] {
      ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);
      grassSkin.getMultiTextureSet().at(0).setTexRepeats(5.0f);
      grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
      grassSkin.setDiffuse(aftrColor4f(1.0f, 0.0f, 0.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
      grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 0.0f)); //Specular color component (ie, how "shiney" it is)
      grassSkin.setSpecularCoefficient(10); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
      });
   wo->setLabel( "Grass" );
   worldLst->push_back( wo );

   
   std::string comicSans = ManagerEnvironmentConfiguration::getLMM() + "/fonts/overwatch_text.ttf";

   //WOGUILabel* label = WOGUILabel::New(nullptr);
   //label->setText("some text");
   //label->setColor( 255,0,0,255 );
   //label->setFontSize(30);//font size is correlated with world size
   //label->setPosition(Vector(0, 1, 0));
   //label->setFontOrientation(FONT_ORIENTATION::foLEFT_TOP);
   //label->setFontPath(comicSans);
   //worldLst->push_back(label);

   WOFTGLString* string = WOFTGLString::New(comicSans,30);//front size should not be confused with world size
   string->getModelT<MGLFTGLString>()->setFontColor(aftrColor4f( 1.0f,0.0f,0.0f,1.0f ));
   string->getModelT<MGLFTGLString>()->setSize(30,10);
   string->getModelT<MGLFTGLString>()->setText("fake overwatch");
   //string->setText("This is only a test.");
   string->rotateAboutGlobalX(Aftr::PI / 2);
   string->setPosition(0, 0, 10);
   worldLst->push_back(string);
   

   //WO* wo2 = WO::New();
   //wo2->setModel(MGLFrustum::New(wo2, .1, 50, 100, 16.0 / 9.0));
   //worldLst->push_back(wo2);
   //wo2->setPosition(0, 0, 50);

   std::vector< Vector > verts;
   std::vector< unsigned int > indices;
   std::vector< aftrColor4ub > colors;

   
   for (size_t i = 0; i < 10000; i++)
   {
      verts.push_back(Vector(rand() % 100, rand() % 100, rand() % 100));
      indices.push_back(i);
      colors.push_back(aftrColor4ub(rand() % 255, rand() % 255, rand() % 255, 255));
   }
  

   WO* pointSet = WO::New();
   //MGLPointCloud* cloud = MGLPointCloud::New(pointSet, this->getCameraPtrPtr(), true, false, false);
   //cloud->setPoints(verts, colors);


   mgl = MGLPointSetShaderAccelerated::New(pointSet, verts, indices, colors, GL_POINTS);
   GLSLShaderPointTesselatorBillboard* shader = dynamic_cast<GLSLShaderPointTesselatorBillboard*>(mgl->getSkin().getShader());
   if (shader != nullptr)
   {
      shader->setDimXY(2, 2);
   }


   pointSet->setModel(mgl);
   worldLst->push_back(pointSet);

   /*
   WOGUITextBox* box = WOGUITextBox::New(nullptr, .5, .1);
   box->setText("a");
   worldLst->push_back(box);
   */
   /*
   WO* star = WO::New(ManagerEnvironmentConfiguration::getLMM() + "models/starDestroyer.stl", Vector(.2, .2, .2) );
   star->getModel()->generateOctreeOfVerticies(10,5);
   star->getModel()->renderOctree = true;
   star->rotateAboutGlobalY(Aftr::PI / 2);
   star->rotateAboutGlobalX(-Aftr::PI / 2);
   
   star->setPosition(Vector(20, 20, 20));
   worldLst->push_back(star);
    */
   //star->getModel()->getNearestPointWhereLineIntersectsMe(rayTail, rayHead, output);

   //worldLst->push_back(string);

   //WOCameraSink* sink = WOCameraSink::New(&this->cam, (WorldList*) this->worldLst, 1000, 10, 10);
   //sink->setPosition(Vector(0, 0, 100));
   //sink->getModel()->getSkin().getMultiTextureSet().at(0) = sink->getFbo()->generateTextureFromFBOTextureOwnsTexDataSharesGLHandle();
  // worldLst->push_back(sink);

   WOCameraSink* sink = WOCameraSink::New(this->getCameraPtrPtr(), (WorldList*) this->worldLst, 100, 16.0, 9.0);
   sink->setPosition(0, 0, 25);
   worldLst->push_back(sink);
}


void GLViewFontTest::createFontTestWayPoints()
{
}
