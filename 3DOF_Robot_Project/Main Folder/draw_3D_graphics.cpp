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
#include "ran.h"

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
//ofstream fout("debug.csv");

void draw_3D_graphics()
{
	//Initialization for each mesh
	static mesh m1("arm1.x");
	static mesh m2("arm2.x");
	static mesh m3("arm3.x");
	static mesh objm1("plane.x");
//	static mesh objm2("car.x");
	static mesh objm3("car.x");			//Change to another object to pick up
	static mesh persm1("car.x");		//Change to Person
//	static mesh bgm("starneb2.jpg");	//See X_file_library Week 7
//	static mesh floorm("track1.x");
	static mesh end_em("arm_grabber.x");
	static mesh goalm("84-GMC-truck.x");
	goalm.Scale = 1.0;
	goalm.Roll_0 = PI / 2;
	end_em.Roll_0 = PI / 2;				//Offset
	end_em.X_0 = -5.5;					//Offset
	persm1.Pitch_0 = 5* PI/ 2;


	//Initialization for arm and object objects
	static Arm arm1(16.5, 10.0, &m1, 0.0, 0.0, 0.0, 0.0, 0.0, PI / 2);				//You can keep adding max of 8 mesh files.
	static Arm arm2(16.5, 10.0, &m2, 0.0, 0.0, 16.5, -PI/6, arm1.yaw, PI / 2);
	static Arm arm3(21.0, 10.0, &m3, 0.0, 0.0, 0.0, PI/6, 0.0, PI / 2);
//	static ObjectWorld w1(1, &objm1, 1, &objm2);
	static Object obj1(3.0, &objm1, 30.0, -20.0, 20.0, 0.0, 0.0, PI / 2);
	static Object obj2(3.0, &objm1, -20.0, 20.0, 0.0, 0.0, 0.0, PI / 2);
	static Object pers1(3.0, &persm1, 20.0, 20.0, 0.0, 0.0, 0.0, PI / 2);
//	static Body bg(&bgm, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static Body end_effector(&end_em, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static Body goal_truck(&goalm, -40.0, -20.0, 0.0, 0.0, 0.0, 0.0);
//	static Body floor(&floorm, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

//TO-DO: Group all objects into an array so we can use for-loops to cycle through them when we need (pickup, fall, draw, ...)

	//Set End_effector Position
	end_effector.Px = arm3.Px + (arm3.length+5) * cos(arm3.pitch)*cos(arm1.yaw);
	end_effector.Py = arm3.Py + (arm3.length+5) * cos(arm3.pitch)*sin(arm1.yaw);
	end_effector.Pz = arm3.Pz + (arm3.length+5) * sin(-arm3.pitch);
	end_effector.pitch = arm3.pitch;
	end_effector.yaw = arm1.yaw;

	//Set Dependent Arm Positions
	arm2.yaw = arm1.yaw;
	arm3.Px = 16.5 * cos(arm2.pitch) * cos(arm1.yaw);
	arm3.Py = 16.5 * cos(arm2.pitch) * sin(arm1.yaw);
	arm3.Pz = 16.5 - 16.5 * sin(arm2.pitch);
	arm3.yaw = arm1.yaw;

	//Person Positions
	pers1.yaw = atan2(pers1.Py - end_effector.Py, pers1.Px - end_effector.Px);

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

	//On-Screen text
	static double objects_in_goal = 0.0;
	text_xy("Place the objects inside the truck!", 10.0, 10.0, 20);
	text_xy("Commands: ", 10.0, 970.0, 10);
	text_xy("W-A-S-D-Q-E = Move Arms", 10.0, 1000, 10);
	text_xy("V = Toggle POV, G = Grab, R = Reset Objects", 10.0, 1030, 10);
	text_xy("Goal = 3", 10.0, 50.0, 20);
	text_xy("Current =", 10.0, 90.0, 20);
	text_xy("Hint: You are an object!", 1650.0, 1000.0, 10);
	text_xy("Press 1 or 2 to auto zoom on objects", 1650.0, 1030.0, 10);
	text_xy(objects_in_goal, 135.0, 90.0, 20);

	if (KEY(0x52))	//KEY: R
	{
		obj1.Px = 30.0; obj1.Py = -20.0; obj1.Pz = 30.0; obj1.pitch = 0.0; obj1.yaw = 0.0; obj1.roll = PI / 2; obj1.previous_z = obj1.Pz;  obj1.is_grabbed = false;
		obj2.Px = -20.0; obj2.Py = 20.0; obj2.Pz = 0.0; obj2.pitch = 0.0; obj2.yaw = 0.0; obj2.roll = PI / 2; obj2.previous_z = obj2.Pz; obj2.is_grabbed = false;
		pers1.Px = 20.0; pers1.Py = 20.0; pers1.Pz = 0.0; pers1.pitch = 0.0; pers1.yaw = 0.0; pers1.roll = PI/2; pers1.is_grabbed = false;
	}

	static bool third_POV = true;

	if (KEY(0x56))		//KEY: V
	{
		third_POV = !third_POV;									//Toggle view point
		Sleep(200);
	}

	if (third_POV)
	{	
		double eye_point[3 + 1] = { -1.0, 60.0, 60.0, 30.0 };		//Third Person
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
	
//	fout << t << "," << T << "," << fps << "\n";


//TO-DO:	Figure out if we can put these inputs in the Arm class somehow

	//Locate Objects with Keystrokes
	static double ObjTheta[3 + 1];
	if (KEY(0x31))		//KEY: 1
	{
		double obj_dist = sqrt(abs((obj1.Px - end_effector.Px) * (obj1.Px - end_effector.Px) + (obj1.Py - end_effector.Py) * (obj1.Py - end_effector.Py) + (obj1.Pz - end_effector.Pz) * (obj1.Pz - end_effector.Pz)));
		locateObject(ObjTheta, obj1.Px, obj1.Py, obj1.Pz);
		text_xy("Located Object1 at: ", 700.0, 250.0, 14);
		text_xy(obj1.Px, 900, 250, 14);
		text_xy(obj1.Py, 970.0, 250.0, 14);
		text_xy(obj1.Pz, 1040.0, 250.0, 14);
		text_xy("Objective distance: ", 700.0, 300.0, 14);
		text_xy(obj_dist, 900.0, 300.0, 14);
	}
//TO-DO:	make obj-dist a function or member var for end_effector

	else if (KEY(0x32))//KEY: 2
	{
		locateObject(ObjTheta, obj2.Px, obj2.Py, obj2.Pz);
		text_xy("Located Object2 at: ", 700.0, 250.0, 14);
		text_xy(obj2.Px, 900, 250, 14);
		text_xy(obj2.Py, 970.0, 250.0, 14);
		text_xy(obj2.Pz, 1040.0, 250.0, 14);
	}
	else if (KEY(0x33))//KEY: 3
	{
		locateObject(ObjTheta, pers1.Px, pers1.Py, pers1.Pz);
		text_xy("Located Object3 at: ", 700.0, 250.0, 14);
		text_xy(pers1.Px, 900, 250, 14);
		text_xy(pers1.Py, 970.0, 250.0, 14);
		text_xy(pers1.Pz, 1040.0, 250.0, 14);
	}


	//Start of Simulation
	dt = 0.00003;
	for (int i = 0; i < 300; i++)
	{
		sim_step(dt, arm1.yaw, arm2.pitch, arm3.pitch, ObjTheta);
//		if (//fish not on top of truck)
		obj1.sim_roam(dt);
		obj2.sim_roam(dt);
//		else
//		obj1.sim_fall(dt);
//		obj2.sim_fall(dt);
		pers1.sim_fall(dt);
	}

//TO-DO: Fish on top of truck

	resolveCollision(obj1, obj2);
	resolveCollision(obj2, pers1);
	resolveCollision(obj1, pers1);

	//Point to object 1, object 2
/*	dt2 = 0.002;
	double objTheta[3 + 1] = { 0.0 };
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
	*/
//TO-DO:	Change locate object to velocity input

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
	end_effector.draw();
	goal_truck.draw();

	//Draw Background
//	bg.draw();
//	floor.draw();

	//Draw Objects
	obj1.draw();
	obj2.draw();
//	w1.draw();
	pers1.draw();
}

