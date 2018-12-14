/*
3DOF Robot MECH 415 Project 

By:
Annie Fong	ID#:
Vivek Patel ID#:
Jeremy Fong ID#: 21952250

//TO-DO: Add a description

Formatted for full screen

*/

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <strstream>
#include <iomanip>
#include <windows.h>		// for keyboard input

// user defined functions
#include "timer.h"			// for measuring time
#include "rotation.h"		// for computing rotation functions
#include "3D_graphics.h"	// for DirectX 3D graphics
#include "my_project.h"

const double PI = atan(1) * 4;

// 3D graphics window size in pixels
int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 1500;
int HEIGHT_MAX = 1000;

// background colour for the scene
float BACK_R = 0.0f;	// red colour component (0 to 1)
float BACK_G = 0.0f;	// green colour component (0 to 1)
float BACK_B = 0.5f;	// blue colour component (0 to 1)

// default min and max viewing distances.
// objects closer than VMIN and farther than VMAX are not drawn (ie cannot be seen).
// note the ratio of VMAX/VMIN should be less than 10,000 for most graphics cards.
double VMIN = 0.25;		// units of m
double VMAX = 1000.0;	// units of m

// global variable for keyboard input
extern unsigned char Key_input[256];

using namespace std;

// define a global file output stream after namespace
ofstream fout("debug.csv");

void draw_3D_graphics()
{
	//Initialization for each mesh
	static double Px, Py, Pz, pitch, yaw, roll;
	static double Px2, Py2, Pz2, pitch2, yaw2, roll2;
	static double Px3, Py3, Pz3, pitch3, yaw3, roll3;
	static double Bgx, Bgy, Bgz, Bgpitch, Bgyaw, Bgroll;
	static double Pxobj1, Pyobj1, Pzobj1, pitchobj1, yawobj1, rollobj1;
	static double Pxobj2, Pyobj2, Pzobj2, pitchobj2, yawobj2, rollobj2;
	static int init = 0;

//TO-DO: Make a Class and an object for all of these

	static double t;											// clock time from t0
	static double t0;											// initial clock time

	static double T, fps, tp, dt, dt2;							// dt used for sim stepping; dt2 used to move arm

	static mesh m1("arm1_test.x");								//the mesh here gets constructed only once since  its a static local variable, so no need to put in the initialization section 
	static mesh m2("arm2.x");									//You can keep adding max of 8 mesh files.
	static mesh m3("arm2.x");
	static mesh bg("background.x");
	static mesh obj1("plane.x");
	static mesh obj2("car.x");

// TO-DO:	Change Background and set it to the proper distance
// TO-DO2:	Make a "person.x" 
// TO-DO3:	Use "person.x" to switch first person and third person views

	// initalization section
	if(!init) {

		// put your initialization tasks here
		// such as opening files, allocating dynamic memory
		// time consuming initialization calculations, etc.

		// note: if you have an object with dynamic memory
		// you can make it a static local variable
		// and the constructor will only get called (along with 
		// new) once
		pitch2 =	PI / 12 - PI / 6;							//PI/12 is added for neutral position, PI/6 is 30 degrees upwards
		pitch3 =	-PI / 12 + PI / 6;							//-PI/12 is added for neutral position, PI/6 is 30 degrees downwards
		Bgx =		-200.0;
		Bgy =		-200.0; 
		Bgz =		0.0; 
		Bgpitch =	0.0; 
		Bgyaw =		0.0;
		Bgroll =	0.0;
	
//TO-DO: Possibly use a constructor (overloaded?) to initialize these

		t0 = high_resolution_time();							// initial clock time (s)
		tp = 0.0;												// previous time

		init = 1;
	}

	double eye_point[3 + 1] =	 { -1.0, 50.0, 50.0, 30.0 };	//Components of eye location
	double lookat_point[3 + 1] = { -1.0, 0.0, 0.0, 20.0 };		//Components of look at point
	double up_dir[3 + 1] =		 { -1.0, 0.0, 0.0, 1.0 };		//Components of up directioni
	double fov =				 PI / 4;						//Field of view (default: 45deg)

	set_view(eye_point,lookat_point,up_dir,fov);

	//Axes in meters (x = red, y = green, z = blue)
	draw_XYZ(3.0);

	//Should we use high_resolution_time()?
	t =		high_resolution_time() - t0;						//Time since the program started (s)
	T =		t - tp;												//Calculate dt frame or period
	fps =	1 / T;												//Frame per second
	tp =	t;													//Update previous time
	
	fout << t << "," << T << "," << fps << "\n";

	//Start of Simulation
	dt = 0.00003;
	for (int i = 0; i < 100; i++) sim_step(dt, yaw, pitch2, pitch3); // fast and stable
//	/\/\/\/\/\/\ - NEED THIS FOR THE PHYSICS/MATH - /\/\/\/\/\/\

	//Set Position based on simulations

	//Pick-up object 1 position
	Pxobj1 =	30.0;
	Pyobj1 =	0.0;
	Pzobj1 =	30.0;
	pitchobj1 = 0.0;
	yawobj1 =	0.0;
	rollobj1 =	0.0;

	//Pick-up object 2 position
	Pxobj2 =	-25.0;
	Pyobj2 =	15.0;
	Pzobj2 =	0.0;
	pitchobj2 = 0.0;
	yawobj2 =	0.0;
	rollobj2 =	PI / 2;

	//Point to object 1, object 2
	dt2 = 0.004;
	double objTheta[3 + 1] = { 0.0 };
	if (KEY(0x31))
	{
		locateObject(objTheta, Pxobj1, Pyobj1, Pzobj1);		//Finds the goal (Object)
		pointAtObject(dt2, objTheta, yaw, pitch2, pitch3);	//Moves towards the goal in increments
	}
	if (KEY(0x32))
	{
		locateObject(objTheta, Pxobj2, Pyobj2, Pzobj2);
		pointAtObject(dt2, objTheta, yaw, pitch2, pitch3);
	}

//TO-DO: Make it move to the positioning instead of instantly pointing at it

	//Arm 1 positions
	Px =	0.0;
	Py =	0.0;
	Pz =	0.0;
//	yaw =	0.0;											//Yaw is updated by simulation
	pitch = 0.0;
	roll =	PI / 2;

	//Arm 1 positions
	Px2 =	0.0;
	Py2 =	0.0;
	Pz2 =	16.5;											//L base ~ 16.5
	yaw2 =  yaw;
//	pitch2 = PI/12 - PI/6;									//Pitch2 is updated by simulation
	roll2 = 0.0;

	//For arm m3
	Px3 =	17.5 * cos(pitch2 - PI / 12) * cos(yaw);
	Py3 =	17.5 * cos(pitch2 - PI / 12) * sin(yaw);
	Pz3 =	17.5 - 17.5 * sin(pitch2 - PI / 12);			//L Arm ~ 17.5; negative is upwards again due to pitch; remove neutral
	yaw3 =	yaw;
//	pitch3 = PI/12;											//Pitch3 is updated by simulation
	roll3 = 0.0;

//TO-DO:	Add Collision detection to stop movements (arms to arms & arms to floor)
//TO-DO2:	Add another mesh for "hand" or "magnet" to "grab" objects
//TO-DO3:	Add another mesh object to be grabbed by the arm (a crate or whatever)
//TO-DO4:	Add grabbing mechanism (if (touching object && press a button) stick object to arm)
//			(if (object stuck to arm && press a button) release object, object falls to ground)

	//Keyboard IN
	if (KEY(VK_UP))		pitch3 -= PI*dt2 / 2;
	if (KEY(VK_DOWN))	pitch3 += PI*dt2 / 2;
	if (KEY(VK_RIGHT))	yaw += PI*dt2/2;
	if (KEY(VK_LEFT))	yaw -= PI*dt2 / 2;
	if (KEY(0x58))
	{
						pitch2 -= PI*dt2 / 2;				//KEY: z
						pitch3 -= PI*dt2 / 2;				//pitch3 so the arm keeps its position
	}
	if (KEY(0x5A))
	{
						pitch2 += PI*dt2 / 2;				//KEY: x
						pitch3 += PI*dt2 / 2;				//pitch3 - same as above
	}

//TO-DO:	Change the Key inputs to activate force instead of position
//TO-DO2:	Maybe add a button to stabilize (stop all movement and keep "gravity" pull the arms down)

	//Draw 3DOF Robot
	m1.draw(Px, Py, Pz, yaw, pitch, roll);
	m2.draw(Px2, Py2, Pz2, yaw2, pitch2, roll2); 
	m3.draw(Px3, Py3, Pz3, yaw3, pitch3, roll3); 

	//Draw Background
	bg.draw(Bgx, Bgy, Bgz, Bgyaw, Bgpitch, Bgroll);

	//Draw Objects
	obj1.draw(Pxobj1, Pyobj1, Pzobj1, yawobj1, pitchobj1, rollobj1);
	obj2.draw(Pxobj2, Pyobj2, Pzobj2, yawobj2, pitchobj2, rollobj2);
}

