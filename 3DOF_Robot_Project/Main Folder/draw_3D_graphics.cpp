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
int WIDTH_MAX = 1920;
int HEIGHT_MAX = 1080;

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
	static mesh m1("arm1.x");
	static mesh m2("arm2.x");
	static mesh m3("arm3.x");
	static mesh objm1("plane.x");
	static mesh objm2("car.x");
	static mesh objm3("car.x");			//Change to another object to pick up
	static mesh persm1("car.x");		//Change to Person
//	static mesh bgm("starneb2.jpg");	//See X_file_library Week 7
//	static mesh floorm("track1.x");
	static mesh end_em("");

	//Initialization for arm and object objects
	static Arm arm1(16.5, 10.0, &m1, 0.0, 0.0, 0.0, 0.0, 0.0, PI / 2);				//You can keep adding max of 8 mesh files.
	static Arm arm2(16.5, 10.0, &m2, 0.0, 0.0, 16.5, -PI/6, arm1.yaw, PI / 2);
	static Arm arm3(21.0, 10.0, &m3, 0.0, 0.0, 0.0, PI/6, 0.0, PI / 2);
//	static ObjectWorld w1(1, &objm1, 1, &objm2);
	static Object obj1(3.0, &objm1, 30.0, 0.0, 30.0, 0.0, 0.0, 0.0);
	static Object obj2(3.0, &objm2, -20.0, 20.0, 0.0, 0.0, 0.0, PI / 2);
	static Object pers1(3.0, &persm1, 20.0, 20.0, 0.0, 0.0, 0.0, PI / 2);
//	static Body bg(&bgm, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static Body end_effector(&end_em, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//	static Body floor(&floorm, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

//TO-DO: Group all objects into an array so we can use for-loops to cycle through them when we need (pickup, fall, draw, ...)

	//Set End_effector Position
	end_effector.Px = arm3.Px + arm3.length * cos(arm3.pitch)*cos(arm1.yaw);
	end_effector.Py = arm3.Py + arm3.length * cos(arm3.pitch)*sin(arm1.yaw);
	end_effector.Pz = arm3.Pz + arm3.length * sin(-arm3.pitch);
	end_effector.pitch = arm3.pitch;
	end_effector.yaw = arm1.yaw;

	//Set Dependent Arm Positions
	arm2.yaw = arm1.yaw;
	arm3.Px = 16.5 * cos(arm2.pitch) * cos(arm1.yaw);
	arm3.Py = 16.5 * cos(arm2.pitch) * sin(arm1.yaw);
	arm3.Pz = 16.5 - 16.5 * sin(arm2.pitch);
	arm3.yaw = arm1.yaw;

	static int init = 0;
	static double t;											// clock time from t0
	static double t0;											// initial clock time
	static double T, fps, tp, dt, dt2;							// dt used for sim stepping; dt2 used to move arm

// TO-DO:	Change Background and set it to the proper distance
// TO-DO2:	Make a "person.x" 

	// initalization section
	if(!init) {

		// put your initialization tasks here
		// such as opening files, allocating dynamic memory
		// time consuming initialization calculations, etc.

		// note: if you have an object with dynamic memory
		// you can make it a static local variable
		// and the constructor will only get called (along with 
		// new) once
//		pitch2 =	0.0;							//PI/19 is added for neutral position, PI/6 is 30 degrees upwards
//		pitch3 =	0.0;							//-PI/12 is added for neutral position, PI/6 is 30 degrees downwards
//		yaw =		0.0;
//		Bgx =		-200.0;
//		Bgy =		-200.0; 
//		Bgz =		0.0; 
//		Bgpitch =	0.0; 
//		Bgyaw =		0.0;
//		Bgroll =	0.0;
	
//TO-DO: Possibly use a constructor (overloaded?) to initialize these

		t0 = high_resolution_time();							// initial clock time (s)
		tp = 0.0;												// previous time
		init = 1;
	}

	static bool third_POV = true;

	if (KEY(0x56))
	{
		third_POV = !third_POV;									//Toggle view point
		Sleep(200);
	}
		
	if (third_POV)
	{	
		double eye_point[3 + 1] = { -1.0, 50.0, 50.0, 30.0 };		//Third Person
		double lookat_point[3 + 1] = { -1.0, 0.0, 0.0, 20.0 };		
		double up_dir[3 + 1] = { -1.0, 0.0, 0.0, 1.0 };				
		double fov = PI / 4;										
		set_view(eye_point, lookat_point, up_dir, fov);
	}
	else
	{
		double eye_point[3 + 1] = { -1.0, pers1.Px, pers1.Py, pers1.Pz };//First Person
		double lookat_point[3 + 1] = { -1.0, end_effector.Px / 2, end_effector.Py / 2, end_effector.Pz / 2 };
		double up_dir[3 + 1] = { -1.0, 0.0, 0.0, 1.0 };				
		double fov = PI/2;										
		set_view(eye_point, lookat_point, up_dir, fov);
	}


	//Axes in meters (x = red, y = green, z = blue)
	draw_XYZ(3.0);

	//Should we use high_resolution_time()??
	t =		high_resolution_time() - t0;						//Time since the program started (s)
	T =		t - tp;												//Calculate dt frame or period
	fps =	1 / T;												//Frame per second
	tp =	t;													//Update previous time
	
	fout << t << "," << T << "," << fps << "\n";


//TO-DO:	Figure out if we can put these inputs in the Arm class somehow

	//Start of Simulation
	dt = 0.00003;
	for (int i = 0; i < 100; i++)
	{
		sim_step(dt, arm1.yaw, arm2.pitch, arm3.pitch);
		obj1.sim_fall(dt);
		obj2.sim_fall(dt);
		pers1.sim_fall(dt);
	}

	//Point to object 1, object 2
	dt2 = 0.002;
/*	double objTheta[3 + 1] = { 0.0 };
	if (KEY(0x31))
	{
		locateObject(objTheta, obj1.Px, obj1.Py, obj1.Pz);		//Finds the goal (Object)
		pointAtObject(dt2, objTheta, arm1.yaw, arm2.pitch, arm3.pitch);	//Moves towards the goal in increments
	}
	if (KEY(0x32))
	{
		locateObject(objTheta, obj2.Px, obj2.Py, obj2.Pz);
		pointAtObject(dt2, objTheta, arm1.yaw, arm2.pitch, arm3.pitch);
	}
//TO-DO:	Change locate object to velocity input
*/
	checkPickup(end_effector, obj1);
	checkPickup(end_effector, obj2);

//TO-DO:	Make all object part of a dynamic array (See week 9 - graphics)
//TO-DO2:	Replace checkPickups with world instead of individual objects 
	checkPickup(end_effector, pers1);

//TO-DO:	Add Collision detection to stop movements (arms to arms & arms to floor)
//TO-DO2:	Add another mesh for "hand" or "magnet" to "grab" objects
//TO-DO3:	Add another mesh object to be grabbed by the arm (a crate or whatever)
//TO-DO4:	Add grabbing mechanism (if (touching object && press a button) stick object to arm)
//			(if (object stuck to arm && press a button) release object, object falls to ground)
//TO-DO5:	Maybe add a button to stabilize (stop all movement and keep "gravity" pull the arms down)

	//Draw 3DOF Robot
	arm1.draw();
	arm2.draw();
	arm3.draw();

	//Draw Background
//	bg.draw();
//	floor.draw();

	//Draw Objects
	obj1.draw();
	obj2.draw();
//	w1.draw();
	pers1.draw();
}

