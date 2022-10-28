//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.0.0 $Rev: 265 $
*/
//===========================================================================
// AUTHOR: SILVIA MARCHESOTTI 
//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <Mmsystem.h>
#include <conio.h>
#include <math.h>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <ctime>


#pragma comment(lib, "Winmm.lib" )
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "Motor.h" 
#include "ParallelPort.c"

//---------------------------------------------------------------------------
#define SAVEDATAMAX 10000000    //MAX number of saved data
#define HoldCount 30
//Delay
#define SAMP 50               //Sampling rate in ms
#define TIMELOG 250         //Saving the data every TIMELOG ms 
#define PI 3.1415926535
int motion;

//------------------Trigger Number for each event------------------
//SYSTEM 
HANDLE hSemaphore;
CRITICAL_SECTION m_cs;
CRITICAL_SECTION protect_flagSEPtimer;

#define TRIAL_START_1 ((short) 0x20) 
#define TRIAL_START_2 ((short) 0x30) 
#define TRIAL_START_3 ((short) 0x28) 
#define TRIAL_START_4 ((short) 0x38) 
short TriggerTRIAL_START[4] = {TRIAL_START_1, TRIAL_START_2, TRIAL_START_3, TRIAL_START_4};

#define TRIAL_END ((short) 0x10)  

#define TriggerSEP ((short) 0x42) 


//---------------------------------------------------------------------------
//For Randomization
SYSTEMTIME st;

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 800;
const int WINDOW_SIZE_H         = 800;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;


// maximum number of haptic devices supported in this demo
const int MAX_DEVICES           = 8;

// size of map
const double SCALE = 0.05;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------
	//------------------- DEFINE HERE THE EXPERIMENT PARAMETERS ------------------- 
	int TrialNum = 16;
	int durationTrial = 15000; //in msec
	int DelayArrayDimension = 0;
	//------------------- DEFINE HERE THE EXPERIMENT PARAMETERS ------------------- 


// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a mesh object used to create the height map
cMesh* cross;
cMesh* check;
cMesh* xSign;

// a light source to illuminate the objects in the virtual scene
cLight *light;
cLight *light_cross; //for flashing cross

//something writtend
//CLabel *initMsg;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW  = 0;
int displayH  = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a table containing pointers to all haptic devices detected on this computer
cGenericHapticDevice* hapticDevices[MAX_DEVICES];

// a table containing pointers to label which display the position of
// each haptic device
//cLabel* labels[MAX_DEVICES];
//cGenericObject* rootLabels;

// number of haptic devices detected
int numHapticDevices = 0;

// table containing a list of 3D cursors for each haptic device
//cShapeSphere* cursors[MAX_DEVICES];
cMesh* cursors[MAX_DEVICES];

// table containing a list of lines to display velocity
cShapeLine* velocityVectors[MAX_DEVICES];

// material properties used to render the color of the cursors
cMaterial matCursorButtonON;
cMaterial matCursorButtonOFF;

// status of the main simulation haptics loop
bool simulationRunning = false;
bool trialRunningHaptic = false;
// root resource path
string resourceRoot;


// damping mode ON/OFF
bool useDamping = false;

// force field mode ON/OFF
bool useForceField = true;

// has exited haptics simulation thread
bool simulationFinished = false;

//global variable, extracted in UpdateMotorLed, 
//then used in updateHaptics
bool currentMvt = false;

// true if the trigger for both SEP 
//and Vibrotactile stimuli has to be sent
bool flagVibro = false;

//used by UpdateMotor() to tell SEPStim() 
//when the trial starts/ends to calculate 
//the lastSEP time	
bool flagSEPtimer = false;

//created to be global to be passed to the SEP_stim function
int motor_num = 5; 

//trial duration
float timeTrial = 0;

int count_SEP = 0;
//variable to store the time of each SEP from the trial's start 
float LastSEP = 0;
float globalLastSEP = 0; 
float LastSEP_array[20] = {0};


// simulation clock
cPrecisionClock threadClock;
cPrecisionClock simClock;
cPrecisionClock LoopClock;
cPrecisionClock TrialClock;
cPrecisionClock SleepCheck;		//check the Sleep() accuracy
cPrecisionClock LastSEPClock;	//time of the SEP from the start of each the trial
cPrecisionClock alterClock;	//time of the SEP from the start of each the trial

//To store Experimental Data
FILE *destinationFile;
FILE *log_file;
FILE *Blog_file;
FILE *expFile;
//Graphic Variables
cMesh* leftHand;
cMesh* rightHand;
cMesh* box;
cMesh* arrow;
void createCross(cMesh* a_mesh );
void createCheck(cMesh* a_mesh );
void createX(cMesh* a_mesh);
const char* textureString;
const char* shapeString;
cTexture2D* texture;

int vertices[6][4];
Motor* motors[4];

//Camera Variable
double cameraAngleH;
double cameraAngleV;
double cameraDistance;
cVector3d cameraPosition;
bool flagCameraInMotion;

//Haptic simulation thread end
bool textWriteFinished = false;
bool textWriteFinishedPos = false;
bool experimentStimulu = false;
bool testStimulu = false;
bool motorTest = false;
bool print = false;
bool trialRunning = false;
bool pressed = false;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;

//Experiment parameters initialization
int counter = 0, responseCounter = 0, testCounter = 0,pedal = 252, response = 0;
int experiment, motor, led;

// Global variables, Time Measurement
LARGE_INTEGER fp;						//  Frequency of counter
LARGE_INTEGER fpExp;						//  Frequency of counter Exp
LARGE_INTEGER scount;					//  Counter value (Start)
LARGE_INTEGER ecount1;					//  Counter value (End)

LARGE_INTEGER scountH;					//  For Haptiuc Loop (Start)
LARGE_INTEGER ecountH;					//  For Haptic Loop (End)

LARGE_INTEGER scountJitter;					//  Counter value (Start)
LARGE_INTEGER ecount1Jitter;					//  Counter value (End)

LARGE_INTEGER sTriggerTime;					//  Counter value (Start)
LARGE_INTEGER eTriggerTime;					//  Counter value (End)

LARGE_INTEGER sLastSEP;					
LARGE_INTEGER eLastSEP;					
LARGE_INTEGER fpSEP;		

LARGE_INTEGER scountEndTrial1;					//  Counter value (Start)
LARGE_INTEGER ecountEndTrial1;					//  Counter value (End)
LARGE_INTEGER fpDuration;					//  Counter value (End)
		
LARGE_INTEGER scountRespTime;					//  Counter value (Start)
LARGE_INTEGER ecountRespTime;					//  Counter value (End)
LARGE_INTEGER fpcountRespTime;					//  Counter value (End)


float currentDurationJitter; 
float currentDuration;
float preDuration =0;
float duration1;
float duration1_log[10][64];
float currentDurationHaptic;
float preDurationHaptic = 0;


//Delay Variable
ULONG DelayedCounter ;
ULONG TimerCounter ;
ULONG TimerCounterStim ;
ULONG TimerCounterResp ;
double DelayedPosition[5000][3];
double DelayedVelocity[5000][3];
double DesiredPosition[3];
double Velocity[3];


//Log of the Data
double TimerCounter_Log[SAVEDATAMAX];
double DesiredPosition_Log[3][SAVEDATAMAX];
double Velocity_Log[3][SAVEDATAMAX];
double ResponcePosition_Log[3][SAVEDATAMAX];

//Passive Motion parameters
int ForceCounter = 0;
int Counter = 0;
double Xincr[360];
double Yincr[360];
double Zincr[360];
cVector3d StartPos;

double Erx = 0;
double Ery = 0;
double Erz = 0;
double PreErx = 0;
double PreEry = 0;
double PreErz = 0;
double Derx = 0;
double Derz = 0;
double Dery = 0;
cVector3d newForce (0,0,0);
cVector3d DesiredPos;

cVector3d rotVecX(1,0,0);
cVector3d rotVecY(0,1,0);
cVector3d rotVecZ(0,0,1);


// VARIABLES FOR TRIAL ORDER
bool DelayOrder[16]; // 0-> no delay applyied; 1-> delayed!
bool ArrowOrder[16]; // 0-> horizontal; 1-> vertical!
bool MvtCongruency[16]; // 0-> congruent movement; 1-> incongruent!
int ExperimentOrder[16]= {0}; //contains the index (1->4) of each trial
float totalDurationEndTrial1 = 0;
float responseTimer = 0;
int TriggerDuration = 100;
float currentDurationEndTrial1 = 0;


//UP DOWN SELECT
bool waitAnswer = false;
int  myAnswer = 0;


// VARIABLES FOR OFFSET 
cVector3d LastPosPrevTrial; //store the last position in the previous trial 
int NumeroTrial;

// VARIABLES FOR STOP AND START 
cBitmap* stop_msg;
   
// VARIABLES FOR FLASHING CROSS
bool flashedTrials[16] = {0};
int timeFlash[16] = {0};


//---------------------------------------------------------------------------
// TRIGGER STUFF
//---------------------------------------------------------------------------
HANDLE hTriggerThread;
HANDLE hMutex;
HANDLE hMutexSEP;
HANDLE hMutexLastSEP;
HANDLE hMutexFlag;
HANDLE hFlagReadDone;

HANDLE hTriggerDone;

int state = 1;
int state1 = 1;
int state2 = 0;
HANDLE threadArray[2];
HANDLE threadArray1[3];

HANDLE hWriteDone;
HANDLE hReadDone;

LPVOID lpParam;

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);
void updateCameraPosition();

// thread to send SEP stimulation
void SEPstim();

//Experiment Function
void updateMotorLed(void);

//Wait Function
void wait (DWORD duration);


//Function which send the trigger
void SendTrigger();
short TriggerValue;
short LocalTriggerValue;

void updateHaptics(void);


int readExperimentOrder();
void DelayTrial(bool, bool, bool);
void randomFlash(int block);
void genRandomOrderTrial(void);
using namespace std;
int repArray[5] = {0}; 
bool prevMvt = false;
bool loadShapeMatTexture(cMesh* leftHand, const char* texture, const char* shape);



//===========================================================================
// MAIN
//===========================================================================	

int main(int argc, char* argv[])
{
	
	InitializeCriticalSection(&m_cs); 

	repArray[0] = 0;
	hFlagReadDone= CreateEvent(NULL, TRUE, FALSE, NULL);
	hWriteDone= CreateEvent(NULL, TRUE, FALSE, NULL);
	hReadDone= CreateEvent(NULL, TRUE, FALSE, NULL);
	hMutexFlag = CreateMutex(NULL, FALSE, NULL);
	hMutexLastSEP= CreateMutex(NULL, FALSE, NULL);
	hTriggerDone = CreateEvent(NULL, TRUE, FALSE, NULL);
	hMutex= CreateMutex(NULL, FALSE, NULL);
	QueryPerformanceFrequency(&fpSEP);	

	
	hMutexSEP = CreateMutex(NULL, TRUE, NULL);
	ReleaseMutex( hMutexSEP );	

    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CHAI 3D\n");
    printf ("Demo: 01-devices\n");
    printf ("Copyright 2003-2009\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Render attraction force\n");
    printf ("[2] - Render viscous environment\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


	//load library for Parallel Port communication
	load_lib();

    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(0.0, 0.0, 0.0);

      // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // define a default position of the camera (described in spherical coordinates)
    cameraAngleH = 0;
    cameraAngleV = 90;
    cameraDistance = 1/SCALE;

    updateCameraPosition();

	camera->setClippingPlanes(0.01, 1/SCALE + 10);


    // create a light source and attach it to the camera
    light = new cLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam

	 // create a light source and attach it to the camera
    light_cross = new cLight(world);
    camera->addChild(light_cross);                   // attach light to camera
    light_cross->setEnabled(false);                   // enable light source
    light_cross->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
    light_cross->setDir(cVector3d(150*SCALE,0*SCALE,23*SCALE));  // define the direction of the light beam

    //-----------------------------------------------------------------------
    // 2D - WIDGETS
    //-----------------------------------------------------------------------

    bool fileload;

    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // read the number of haptic devices currently connected to the computer
    numHapticDevices = handler->getNumDevices();


    // limit the number of devices to MAX_DEVICES
    numHapticDevices = cMin(numHapticDevices, MAX_DEVICES);

    // for each available haptic device, create a 3D cursor
    // and a small line to show velocity

    int i = 0;
    while (i < numHapticDevices)
    {
        // get a handle to the next haptic device
        cGenericHapticDevice* newHapticDevice;
        handler->getDevice(newHapticDevice, i);

        // open connection to haptic device
        newHapticDevice->open();

		// initialize haptic device
		newHapticDevice->initialize();

        // store the handle in the haptic device table
        hapticDevices[i] = newHapticDevice;

        // retrieve information about the current haptic device
        cHapticDeviceInfo info = newHapticDevice->getSpecifications();

	    // increment counter
        i++;
    }


 	// LEFT HAND //
	leftHand = new cMesh(world);
	textureString = "resources/models/hand/skin.bmp";
	shapeString = "resources/models/hand/LeftArmLight.3ds"; 
	fileload = loadShapeMatTexture(leftHand,textureString,shapeString );
	if(!fileload)
		return -1;
	
	leftHand->rotate(cNormalize(rotVecZ),3.14);
	leftHand->rotate(cNormalize(rotVecX),-3.14/11);
	leftHand->rotate(cNormalize(rotVecY),-3.14/22);
	leftHand->setPos(50*SCALE,-190*SCALE,-286*SCALE);
	world->addChild(leftHand);
	

	// RIGHT HAND //
	rightHand = new cMesh(world);
	textureString = "resources/models/hand/skin.bmp";
	shapeString = "resources/models/hand/RightArmLight.3ds"; 
	fileload = loadShapeMatTexture(rightHand,textureString,shapeString );
	if(!fileload)
		return -1;
	rightHand->rotate(cNormalize(rotVecX),3.14/16);
	rightHand->setPos(0,170*SCALE,-295.0*SCALE);
	

	// attach robot to tool
	world->addChild(rightHand);
  
	
	// BOX // 
	box = new cMesh(world);

	textureString = "resources/models/hand/brownOrange.JPG";
	shapeString = "resources/models/hand/LongBox.3ds"; 
	fileload = loadShapeMatTexture(box,textureString,shapeString);
	if(!fileload)
		return -1;
	box->rotate(cNormalize(rotVecZ),3.14/2);
	box->setPos(25*SCALE,-3*SCALE,+33*SCALE);
	box->scale(0.9);
	world->addChild(box);



	// CROSS // 
	cross = new cMesh(world);
	createCross(cross);
	cross->setPos(150*SCALE,0*SCALE,0*SCALE); //(z,x,y)
	cross->rotate(cNormalize(rotVecZ),3.14/2);
	cross->scale(0.7);
	world->addChild(cross);	

	

    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI 3D");

    // create a mouse menu (right button)
    glutCreateMenu(menuSelect);
    glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
    glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
    glutAttachMenu(GLUT_RIGHT_BUTTON);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	cThread* motorLedThread = new cThread();
	motorLedThread->set(updateMotorLed, CHAI_THREAD_PRIORITY_GRAPHICS);

	cThread* SEPThread = new cThread();
    SEPThread->set(SEPstim, CHAI_THREAD_PRIORITY_GRAPHICS);


	threadArray[0] = motorLedThread;
	threadArray[1] = SEPThread;
	WaitForMultipleObjects(2, threadArray, TRUE, INFINITE);


    // start the main graphics rendering loop
    glutMainLoop();
	
    // close everything
    close();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);

}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

	if (key == 'b')
	{
		box->setPos(25*SCALE,-6*SCALE,92*SCALE);
		rightHand->setPos(0,150*SCALE,-225*SCALE);
		leftHand->setPos(0,-175*SCALE,-225*SCALE);

	}

	if (key == 'p')
	{
		pressed = true;
	}

	// --------------------------------------------

	if (key == 'e' || key == 'E')
     {
	   experimentStimulu = true;
    }

	if (key == 't')
	{
		testStimulu = true;
	}

	if (key == 'm')
	{
		motorTest = true;
	}

	if (key == 's')
	{
		
		motion = 0;
	}

    if (key == 'a')
	{
		motion = 1;
	}

	updateCameraPosition();

	if ((key == '4') && (waitAnswer == true))
	{
		myAnswer = 1;
	}

	if ((key == '3') && (waitAnswer == true))
	{
		myAnswer = 2;
	}

}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
            break;
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

	int i=0;
    while (i < numHapticDevices)
    {
        hapticDevices[i]->close();
        i++;
    }

}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
     // render world
    camera->renderView(displayW, displayH);

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------
bool loadShapeMatTexture(cMesh* leftHand,const char* texture,const char* shape)
{
	bool fileload;

	cTexture2D* texture1 = new cTexture2D();
    fileload = texture1->loadFromFile(RESOURCE_PATH(texture));
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
    }

    texture1->setEnvironmentMode(GL_DECAL);
    texture1->setSphericalMappingEnabled(true);

    // load an object file
    fileload = leftHand->loadFromFile(RESOURCE_PATH(shape));
 
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
    }

    // resize tool mesh model
    leftHand->scale(SCALE);

	leftHand->setTexture(texture1, true);
    leftHand->setUseTexture(true, true);
	return fileload;
}

//---------------------------------------------------------------------------

void createX(cMesh* a_mesh)
{
    const double length = 1;
	const double width = 0.05;
    int vertices [2][4];

    // Horizantal
    vertices[0][0] = a_mesh->newVertex( 0, -length , width);
    vertices[0][1] = a_mesh->newVertex( 0, -length , -width);
    vertices[0][2] = a_mesh->newVertex( 0, length,  -width);
    vertices[0][3] = a_mesh->newVertex( 0, length ,  width);

    // Horizantal
    vertices[1][0] = a_mesh->newVertex( 0 ,  -width , -length);
	vertices[1][1] = a_mesh->newVertex( 0 , width , -length);
    vertices[1][2] = a_mesh->newVertex( 0, width , length);
    vertices[1][3] = a_mesh->newVertex(0, -width , length);

    // create triangles
    for (int i=0; i<2; i++)
    {
		a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
		a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    // set material properties to light gray
    a_mesh->m_material.m_ambient.set(0.8, 0, 0.0);
    a_mesh->m_material.m_diffuse.set(1.0, 0.15, 0.5);
    a_mesh->m_material.m_specular.set(1.0, 0.2, 0.8);
    a_mesh->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);
	
}

//---------------------------------------------------------------------------


void createCheck(cMesh* a_mesh)
{
    const double length = 1;
	const double width = 0.1;
    int vertices [2][4];

    // Horizantal
    vertices[0][0] = a_mesh->newVertex( 0, - length - width , 0);
    vertices[0][1] = a_mesh->newVertex( 0, 0, 0);
    vertices[0][2] = a_mesh->newVertex( 0, 0,  width);
    vertices[0][3] = a_mesh->newVertex( 0, - length - width ,  width);

    // Horizantal
    vertices[1][0] = a_mesh->newVertex( 0 ,  0 , 0 );
	vertices[1][1] = a_mesh->newVertex( 0 , 0, 1.5 * (length + width));
    vertices[1][2] = a_mesh->newVertex( 0, -width , 1.5 * (length + width) );
    vertices[1][3] = a_mesh->newVertex(0, -width , 0);

    // create triangles
    for (int i=0; i<2; i++)
    {
		a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
		a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    // set material properties to light gray
    a_mesh->m_material.m_ambient.set(0.0, 0.8, 0.0);
    a_mesh->m_material.m_diffuse.set(1.0, 0.15, 0.5);
    a_mesh->m_material.m_specular.set(1.0, 0.2, 0.8);
    a_mesh->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);
	
}


//---------------------------------------------------------------------------

void updateCameraPosition()
{
    // check values
    if (cameraDistance < 0.1) { cameraDistance = 0.1; }

    // compute position of camera in space
    cVector3d pos(
                            cameraDistance * cCosDeg(cameraAngleH) * cSinDeg(cameraAngleV),
                            cameraDistance * cSinDeg(cameraAngleH) * cSinDeg(cameraAngleV),
                            cameraDistance * cCosDeg(cameraAngleV)
                    );

    // compute lookat position
    cVector3d lookat(0,0,0);

    // define role orientation of camera
    cVector3d up(0.0, 0.0, 1.0);

    // set new position to camera
    camera->set(pos, lookat, up);

    // recompute global positions
    world->computeGlobalPositions(true);

}

//---------------------------------------------------------------------------

void createCross(cMesh* a_mesh)
{
    const double HALFSIZE = 0.6;
	const double lengthToHeight = 0.08;
    int vertices [2][4];

    // Horizantal
    vertices[0][0] = a_mesh->newVertex( - HALFSIZE, 0, -lengthToHeight*HALFSIZE);
    vertices[0][1] = a_mesh->newVertex( HALFSIZE, 0, -lengthToHeight*HALFSIZE);
    vertices[0][2] = a_mesh->newVertex( HALFSIZE, 0,  lengthToHeight*HALFSIZE);
    vertices[0][3] = a_mesh->newVertex( - HALFSIZE, 0,  lengthToHeight*HALFSIZE);

    // Horizantal
    vertices[1][0] = a_mesh->newVertex( -lengthToHeight*HALFSIZE,  0,-HALFSIZE);
    vertices[1][1] = a_mesh->newVertex( lengthToHeight*HALFSIZE, 0, -HALFSIZE);
    vertices[1][2] = a_mesh->newVertex( lengthToHeight*HALFSIZE, 0,  HALFSIZE);
    vertices[1][3] = a_mesh->newVertex( -lengthToHeight*HALFSIZE, 0,  HALFSIZE);

    // create triangles
    for (int i=0; i<2; i++)
    {
		a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
		a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }
	
	a_mesh->m_material.m_ambient.set(1, 1, 1); //white

	
}


//---------------------------------------------------------------------------

void updateMotorLed(void)
{
	hMutex = CreateMutex( NULL, TRUE, NULL ); 

	ReleaseMutex( hMutex );	


	int counter = 0, responseCounter = 0, testCounter = 0, response = 0; //pedal = 252


	int experiment, motor, led;
	double StaticValueX = 0.03433;
	double StaticValueY = 0.0597;
	double StaticValueZ = -0.092;
    cVector3d ResponcePosition;
	cVector3d StimPosition;
	cVector3d Pos;
    

	motors[0] = new Motor("Dev5/ao0", "Dev5/ai0", "Dev5/ctr0", "Dev4/port0");
	motors[1] = new Motor("Dev5/ao1", "Dev5/ai1", "Dev5/ctr1", "Dev4/port0");
	motors[2] = new Motor("Dev4/ao0", "Dev4/ai0", "Dev4/ctr0", "Dev4/port0");
	motors[3] = new Motor("Dev4/ao1", "Dev4/ai1", "Dev4/ctr1", "Dev4/port0");


	genRandomOrderTrial();
	readExperimentOrder();
	int block;
	printf("Insert block index from 0 --> 4: \n");
	cin>>block;
	//randomFlash(block);

	///////////////////////////////////////////////////////////////////////////////////////////////
	//	Print the block's order in a .txt file
	///////////////////////////////////////////////////////////////////////////////////////////////


	char blockID[50];						
	char blockNum[50]; 
	strcpy(blockNum,"Block");
	sprintf(blockID,"%d",block);
	strcat(blockNum,blockID);
	//strcat(fileNamePos,trialIDch);
	strcat(blockNum,".txt");
	printf("block NAME %s\n", blockNum);
					
	if ((Blog_file = fopen(blockNum, "wt")) == NULL) 
	{printf("Error: can't access Objective.txt \n");}

	
			///////////////////////////////////////////////////////////////////////////////////////////////
			//	Save variables
			///////////////////////////////////////////////////////////////////////////////////////////////
					 for (int c = 0; c<TrialNum; c++)
					 {
						 printf("Exp_order %d\n", ExperimentOrder[c]);
					  	   fprintf(Blog_file, "%d ", ExperimentOrder[c] );   //*SAMP
					 }
					//  File close
					 fclose(Blog_file);

			 ///////////////////////////////////////////////////////////////////////////////////////////////
			//End of writing
			///////////////////////////////////////////////////////////////////////////////////////////////


	while(simulationRunning)
	{
		if (motorTest)
		{
			for (int i = 0; i< 4; i++)
			{
				//wait(1500);
				//motors[i]->setVolt(5);
				//wait (30);
				//motors[i]->setVolt(0);
			
				// --------------LED TRIGGER!--------------
				//Out32_a(PPORT_BASE,TriggerMOTOR_pos[i]);
				// ----------------------------------------

			}
			motorTest = false;

		}


		if (experimentStimulu) 
		{
			for (int j=0; j<1; j++) 
			{
			for (int i = 0; i<TrialNum; i++) 
				{   
					
					NumeroTrial = i;
										
					// --------------EXPERIMENT START!--------------
                 	printf ("--------------EXPERIMENT START!--------------\n");					
					int trialID = ExperimentOrder[i];
					printf ("TRIAL START INDEX %d\n", trialID);
					WaitForSingleObject(hMutexSEP, INFINITE); 					
					TriggerValue = TriggerTRIAL_START[trialID-1];
					printf ("-> 0x%01.1X Acquired\n", TriggerValue);
					SendTrigger();
					printf ("-> 0x%01.1X Done\n", TriggerValue);
					printf ("-------------------------\n");


					// -------------- DELAY OR REAL TIME? --------------					
					repArray[trialID] = repArray[trialID] + 1;

					currentMvt = MvtCongruency[i];

					prevMvt = MvtCongruency[i-1];
					bool booldel = DelayOrder[i];

					DelayTrial(booldel, currentMvt, prevMvt);

					wait(1000);

					leftHand->setTransparencyLevel(1);
					rightHand->setTransparencyLevel(1);
					box->setTransparencyLevel(1);
		

					waitAnswer = false;
					myAnswer = 0;

					int totDuration = 0; 
					totalDurationEndTrial1 = 0;
					currentDurationEndTrial1 = 0;
					
			
					QueryPerformanceFrequency(&fpDuration);

					while(true) 
					{

						QueryPerformanceCounter(&scountEndTrial1); //start counting for the trial duration, a senso l'avrei messo nel loop piu' interno, invece funziona cosi'...
						while(true) 
						{

							QueryPerformanceCounter(&ecountEndTrial1);
							currentDurationEndTrial1 = (float)(1. * (ecountEndTrial1.LowPart - scountEndTrial1.LowPart) * 1000.0 / fpDuration.LowPart);
							if (currentDurationEndTrial1 > 1000) {		
								break; 			
							}
						} //while(currentDurationEndTrial1...
				

						totalDurationEndTrial1 = totalDurationEndTrial1 + currentDurationEndTrial1;
						
						//Check if the flash moment has arrived!
						totDuration = ceil(totalDurationEndTrial1);
						
						/*
						if(( flashedTrials[i] != 0)&&(totDuration >= (1000*timeFlash[i]))&&(totDuration <= (1000*timeFlash[i]+900))) {
						
							cross->m_material.m_specular.set(1.0, 1.0, 1.0);				
							light_cross->setEnabled(true);
							wait(300);						
							QueryPerformanceCounter(&scountRespTime);
							cross->m_material.m_specular.set(0.0, 0.0, 0.0);
							light_cross->setEnabled(false);
							
						}
						*/

						if (totalDurationEndTrial1 > durationTrial) { 

							printf("totalDurationEndTrial1:   %f\n",totalDurationEndTrial1);
							break;
						}
						currentDurationEndTrial1 = 0;
						totDuration = 0;
					} //while(totalDurationEndTrial1...



					///////////////////////////////////////////////////////////////////////////////////////////////
					//	Print file
					///////////////////////////////////////////////////////////////////////////////////////////////

					char trialIDch[50];						
					char fileNamePos[50]; 
					strcpy(fileNamePos,"Block");
					sprintf(trialIDch,"%d_%d_%d",block,trialID, repArray[trialID]);
					strcat(fileNamePos,trialIDch);
					strcat(fileNamePos,".txt");
					printf("fILE NAME %s\n", fileNamePos);

						if ((log_file = fopen(fileNamePos, "wt")) == NULL) 
						{printf("Error: can't access Objective.txt \n");}

					///////////////////////////////////////////////////////////////////////////////////////////////
					//	Save title of variable
					///////////////////////////////////////////////////////////////////////////////////////////////
					 fprintf(log_file, "Timer	\t");
					 for (int i = 0; i < 3; i++) {
						 fprintf(log_file, "Position[%d]	\t", i);
					  }
					 fprintf(log_file, "\n");

					///////////////////////////////////////////////////////////////////////////////////////////////
					//	Save variables
					///////////////////////////////////////////////////////////////////////////////////////////////
					 for (int i = 1; ((i < TimerCounter) && (i < SAVEDATAMAX)); i++)
					 {
						  if (TimerCounter_Log[i] != NULL) {
							   fprintf(log_file, "%lf	\t", TimerCounter_Log[i] );   //*SAMP

							   for (int j = 0; j < 3; j++) {
								   fprintf(log_file, "%lf	\t", DesiredPosition_Log[j][i]);
							   }
							   fprintf(log_file, "\n");
						  }
					 }

					//  File close
					fclose(log_file);


					printf("TRIAL DONEEEEEEEEEEEEEE\n");
					
					if (booldel == 1) { printf("DELLLAAAAAYYYY\n"); }
					trialRunningHaptic = false;				



					WaitForSingleObject(hMutexSEP, INFINITE); 					
					TriggerValue = TRIAL_END;
					printf ("-> 0x%01.1X Acquired\n", TriggerValue);
					SendTrigger();	
					printf ("-> 0x%01.1X Done\n", TriggerValue);

					printf ("-------------------------\n");

		
					
					leftHand->setTransparencyLevel(0);
					rightHand->setTransparencyLevel(0);
					box->setTransparencyLevel(0);

				

					for (int isi = 0; isi < 4; isi++) 
					{
						wait(1000);
					}

					PlaySound("D:\WINDOWS\Media\ding.WAV", NULL, SND_SYNC);

					
			///////////////////////////////////////////////////////////////////////////////////////////////
			//	Print file!!
			///////////////////////////////////////////////////////////////////////////////////////////////
		 

			} //for(i trialNum) 

			printf("Experiment Finished\n");
			experimentStimulu = false; //end of the session (opp. to 'e')
			simulationRunning = false;
			
			} //for j 

		}//experimentStimulu
		

	}//while(stimRunning)


	CloseHandle(hMutex);
	textWriteFinished = true;
	simulationFinished = true;

	exit(0);

}


//---------------------------------------------------------------------------


void wait (DWORD duration)
{
	LARGE_INTEGER fpWait;
	LARGE_INTEGER scountWait;					//  Counter value (Start)
	LARGE_INTEGER ecountWait;					//  Counter value (End)
	QueryPerformanceFrequency(&fpWait);
	QueryPerformanceCounter(&scountWait);
	while (1) {
		QueryPerformanceCounter(&ecountWait);
		float currentDurationWait = (float)(1. * (ecountWait.LowPart - scountWait.LowPart) * 1000.0 / fpWait.LowPart);
		if (currentDurationWait >= duration) {
			//printf ("EXIT THE WAIT ----------------------\n"); 
			break;
		}				
	}
}


//---------------------------------------------------------------------------

void updateHaptics(void)
{

	while(simulationRunning)
	{   
		cVector3d posDum;
		while (trialRunningHaptic)
		{
			
			  LoopClock.start();
			  QueryPerformanceFrequency(&fp);
			  QueryPerformanceCounter(&scount);		

			  // for each device
				int i=0;
				while (i < numHapticDevices)
				{    
		            cVector3d SetP;
					cVector3d SetPR;
					cVector3d SetPB;

					cVector3d Offset1; //left hand
					cVector3d Offset2; //right hand
					cVector3d Offset3; //box

					
					Offset1.x = 50*SCALE; 
					Offset1.y = -190*SCALE; 
					Offset1.z = -306*SCALE; // -20 original setPos

					Offset2.x = 0;
					Offset2.y = 170*SCALE; 
					Offset2.z = -305.0*SCALE; // -10 original setPos

					Offset3.x = 25*SCALE;
					Offset3.y = -3*SCALE; 
					Offset3.z = 13*SCALE; // -20 original setPos

					
					SetP.x = DelayedPosition[DelayedCounter][0];
					SetP.y = DelayedPosition[DelayedCounter][1];
					SetP.z = DelayedPosition[DelayedCounter][2];
				


					SetP.mul(1/SCALE);
					SetPR = SetP;
					SetPB = SetP;
					SetP.add(Offset1);
					leftHand->setPos(SetP);
					SetPR.add(Offset2);
					rightHand->setPos(SetPR);
					SetPB.add(Offset3);
					box->setPos(SetPB);

					// read position of haptic device  
					cVector3d newP;
					

					LastPosPrevTrial = newP;
					hapticDevices[i]->getPosition(newP); //take position for the next time!


					newP.mul(1.5); //MULTIPLYING FACTOR (realtà -> VR) 
					newP.add(0.1411,-0.0829,0.1450);

						
					if (currentMvt == true) {
						newP.add(0,-0.07,0.08); 
						DesiredPosition[0] = newP.x; //z
						DesiredPosition[1] = newP.z; //x
						DesiredPosition[2] = newP.y; //y
					}

					else {

						newP.add(0,-0.04,0.05); //(0, +left, +down)
						DesiredPosition[0] = newP.x;
						DesiredPosition[1] = newP.y;
						DesiredPosition[2] = newP.z;		
					}

				//LogData
					
				  	if (TimerCounter < SAVEDATAMAX)
					 {
						TimerCounter_Log[TimerCounter] = TimerCounter;
						for (i = 0; i < 3; i++) 
						{DesiredPosition_Log[i][TimerCounter] = DesiredPosition[i];
						}
					  }
				
					//Shift Data
					   for (int i = 0; i < 3; i++) 
						   { DelayedPosition[DelayedCounter][i] = DesiredPosition[i];
					   }
					
				
					DelayedCounter++;
					TimerCounter++;

					if (DelayedCounter > DelayArrayDimension) 
						{ DelayedCounter = 0;
						}
					i++;
					} //while(i < numHapticDevices)

		      
				QueryPerformanceCounter(&ecount1);
				currentDuration = (float)(1. * (ecount1.LowPart - scount.LowPart) * 1000.0 / fp.LowPart);
				double error = (currentDuration - preDuration);
				preDuration = currentDuration;
		      
				wait((20-currentDuration));
		       

				} //while(trialRunningHaptic)
	} //while(simulationRunning ...

} 

//---------------------------------------------------------------------------

int readExperimentOrder() 
{
	char arra[32][32];
	char new_arra[32][32];
	char line[32]; 
	int ord = 0;


	int sizearra = sizeof(ExperimentOrder);

	for (int p=0; p<sizearra; p++)
	{
		
		switch (ExperimentOrder[p]) {

			//RT - Horizontal
			case 1: {	
				DelayOrder[p] = 0;
				MvtCongruency[p] = 0;
				break;
					}

			//Delayed - Horizontal
			case 2: {
				DelayOrder[p] = 1;
				MvtCongruency[p] = 0;
				break;
					}
			//RT - Vertical
			case 3: {
				DelayOrder[p] = 0;
				MvtCongruency[p] = 1;
				break;
					}
			//Delayed - Vertical
			case 4: {
				DelayOrder[p] = 1;
				MvtCongruency[p] = 1;
				break;
					}	
		}

		ord++;
	}

	for (int c=0; c<16; c++) {
	printf("Trial ID= %d\n",ExperimentOrder[c]);
	}

	return 0;

}


//---------------------------------------------------------------------------

void DelayTrial(bool DelayBool, bool currentMvt, bool prevMvt) {
//fill the buffer (being 30 element or 1 element big) with the same values
//taken from hapticDevices-> getposition if it's the first trial, otherwise 
//from the last position of the previous trial 

	DelayArrayDimension = HoldCount*DelayBool;

	DelayedCounter = 0;
    TimerCounter = 0;
	

	//buffer dei primi tot msec
	int p = 0;
    if (p < numHapticDevices)
	{
	  cVector3d newPosition;
	  
	  hapticDevices[p]->getPosition(newPosition); 

	  DesiredPosition[0] = newPosition.x; 
	  DesiredPosition[1] = newPosition.y;
	  DesiredPosition[2] = newPosition.z;  
	  
	
	  //------previously used for storing previous trial final position which here becames the starting one
	  if (NumeroTrial == 0) {
		  hapticDevices[p]->getPosition(newPosition); }
	  else {
		  newPosition = LastPosPrevTrial;}
	  //----------------------------------------------------------------------------------------------------
	 
	  
	  
	  if(((currentMvt == false) && (prevMvt == true))||((currentMvt == true) && (prevMvt == false))) {

		  DesiredPosition[0] = newPosition.x; 
		  DesiredPosition[1] = newPosition.z;
		  DesiredPosition[2] = newPosition.y;
	  }

	  else {

		  DesiredPosition[0] = newPosition.x; 
		  DesiredPosition[1] = newPosition.y;
		  DesiredPosition[2] = newPosition.z;
	  }

	  
  
	  for (int i = 0; i <= DelayArrayDimension; i++)
	  { 
	     for (int j = 0; j < 3; j++) {
			DelayedPosition[i][j] = DesiredPosition[j];

			}     
	  }
	}
	 
	trialRunningHaptic = true;

}



//---------------------------------------------------------------------------

void SEPstim()
{
	while(simulationRunning) // // experimentStimulu = true;
  	{ 
	  //printf ("Inside SEPstim function\n");
	  // ------------ TRIGGER CODE ------------ 		
	  WaitForSingleObject(hMutexSEP, INFINITE); 
	  TriggerValue = TriggerSEP;
	  //printf ("-> 0x%01.1X Acquired\n", TriggerValue);
	  SendTrigger();
	  //printf("SEP-> SendTrigger just called\n");
	  wait(500); //wait for the next trigger
	}
}



//---------------------------------------------------------------------------

void SendTrigger(void)
{
	short LocalTriggerValue = TriggerValue;
	
	float currentTh = 0;
	threadClock.reset();
	
	Out32(PPORT_BASE,LocalTriggerValue);	//Send trigger		
	threadClock.start();
	//printf ("--------------> Trigger: 0x%01.1X\n", TriggerValue);
	wait(50);
	threadClock.stop();
	currentTh = threadClock.getCurrentTimeSeconds();

	//printf ("-> Time for 0x%01.1X: %f\n", LocalTriggerValue, currentTh);
	Out32(PPORT_BASE,0x00);	//Trigger reset		
	wait(50);
	ReleaseMutex( hMutexSEP );					

}


//---------------------------------------------------------------------------

void genRandomOrderTrial(void) {

	int ExperimentOrder_cp[16]= {1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4};


	for (int s=0; s<(TrialNum-1); s++) {

		int r = s + (rand()%(TrialNum-s));
		int temp = ExperimentOrder_cp[s];
		ExperimentOrder_cp[s]= ExperimentOrder_cp[r];
		ExperimentOrder_cp[r] = temp;

	}

	printf("ExperimentOrder cippini: ");
	for (int f = 0; f< TrialNum; f++) {
		printf("%d ",ExperimentOrder_cp[f]);
	}

		printf("\n");

		for (int z=0 ; z<TrialNum; z++) {
			ExperimentOrder[z] = ExperimentOrder_cp[z];
		
		}
		
}


//---------------------------------------------------------------------------

void randomFlash(int block) {

	int numFlash;

	switch(block) {
	
		case 0: {
			numFlash = 4;
			break;}
				
		case 1: {
			numFlash = 2;
			break;}

		case 2: {
			numFlash = 3;
			break;}

		case 3: {
			numFlash = 1;
			break;}

		case 4: {
			numFlash = 3;
			break;}
	}

	int flashedSeconds[6] = {0}; 
	int counter = 0;
	int flashedTrialsIdx[6] = {0}; 
	int flashTr = 0;
	double cicc=0;
	bool hasRepetition = false;
	int first = 0;

	srand((unsigned)time(0));
    
	while(counter < numFlash)
     {
	 	do {
				
			hasRepetition = false; 
			flashedTrialsIdx[counter] = rand()%16;//flashedTrialsIdx contains at which instant of time the flash will occur
			 
			 for (int z = 0; z < counter; z++) {

				 if (flashedTrialsIdx[counter] == flashedTrialsIdx[z]) {
					 hasRepetition = true; 					
				 }								 
			 } 
		 }while(hasRepetition);
		 
		 
	
		 cicc =(rand()%durationTrial)/1000;
		 flashedSeconds[counter] =  ceil(cicc)+1;
		 counter++;
	 	 hasRepetition = false;
	
     }


	printf("NumFlash = %d\n", numFlash);

	for(int u = 0; u < numFlash; u++) {
		printf("Trial flashed %d on Time:%d\n", flashedTrialsIdx[u], flashedSeconds[u]);
	}


	//fill booleans array which will be read in the updateMotorLed
	for (int idx = 0; idx<numFlash; idx++) {		
		flashedTrials[flashedTrialsIdx[idx]] = 1;
		timeFlash[flashedTrialsIdx[idx]] = flashedSeconds[idx]; 
	}


}

