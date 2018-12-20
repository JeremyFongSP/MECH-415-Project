/*
3DOF Robot MECH 415 Project 

By:
Annie Fong	ID#: 27398034
Vivek Patel ID#: 27532377
Jeremy Fong ID#: 21952250

Description:
3DOF Arm without gravity, using inverse kinematics for automatic targeting
Inputs are velocity, state variables are positions of the three links
Presented as a game of catching fish

Formatted for a full screen of 1920x1080
*/

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <strstream>
#include <iomanip>
#include <windows.h>

// user defined functions
#include "timer.h"
#include "rotation.h"
#include "3D_graphics.h"
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
float BACK_G = 0.2f;	// green colour component (0 to 1)
float BACK_B = 0.5f;	// blue colour component (0 to 1)

double VMIN = 0.25;		// units of m
double VMAX = 1000.0;	// units of m

using namespace std;

// define a global file output stream after namespace
//ofstream fout("debug.csv");

void draw_3D_graphics()
{
	//Initialization for each mesh
	static mesh m1("arm1.x");
	static mesh m2("arm2.x");
	static mesh m3("arm3.x");
	static mesh persm1("3dm-r-r2-d2.x");
	static mesh fishm("fish1.x");
	static mesh goalm("cage.x");

	//Proper Scaling/Orientation
	persm1.Scale = 0.2;
	goalm.Scale = 5.0;
	goalm.Roll_0 = PI / 2;
	persm1.Pitch_0 = 5* PI/ 2;

	//Initialization
	//3DOF ROBOT
	static Arm arm1(16.5, 10.0, &m1, 0.0, 0.0, 0.0, 0.0, 0.0, PI / 2);
	static Arm arm2(16.5, 10.0, &m2, 0.0, 0.0, 16.5, -PI/6, arm1.yaw, PI / 2);
	static Arm arm3(21.0, 10.0, &m3, 0.0, 0.0, 0.0, PI/6, 0.0, PI / 2);
	//OBJECTS
	static FishWorld fishes(2);
	static Object pers1(3.0, &persm1);
	static Body end_effector(&m1);								//m1 is placeholder if ever we want to add an actual model
	static Body cage(&goalm, -40.0, -20.0, 0.0, 0.0, 0.0, 0.0);

	//Set End_effector Position
	end_effector.Px = arm3.Px + (arm3.length+5) * cos(arm3.pitch)*cos(arm1.yaw);
	end_effector.Py = arm3.Py + (arm3.length+5) * cos(arm3.pitch)*sin(arm1.yaw);
	end_effector.Pz = arm3.Pz + (arm3.length+5) * sin(-arm3.pitch);
	end_effector.pitch = arm3.pitch;
	end_effector.yaw = arm1.yaw;	

	//Set Dependent Arms Position
	arm2.yaw = arm1.yaw;	
	arm3.Px = 16.5 * cos(arm2.pitch) * cos(arm1.yaw);
	arm3.Py = 16.5 * cos(arm2.pitch) * sin(arm1.yaw);
	arm3.Pz = 16.5 - 16.5 * sin(arm2.pitch);
	arm3.yaw = arm1.yaw;

	//Person Position
	pers1.yaw = PI + atan2(pers1.Py - end_effector.Py, pers1.Px - end_effector.Px);

	//Initialize Time and Score
	static int init = 0;
	static double t;									
	static double t0;
	static double T, fps, tp, dt, dt2;			
	static int score = 0;
	static int money = 0;

	//Initalization for Objects + Reset Section
	if(!init) {
		fishes.pf[1]->Px = 15.0;
		fishes.pf[1]->Py = -15.0;
		fishes.pf[1]->Pz = 20.0;
		fishes.pf[1]->yaw = 0.0;
		fishes.pf[1]->roll = 0.0;
		fishes.pf[1]->pitch = 0.0;
		fishes.pf[1]->previous_x = fishes.pf[1]->Px;
		fishes.pf[1]->previous_y = fishes.pf[1]->Py;
		fishes.pf[1]->previous_z = fishes.pf[1]->Pz;
		fishes.pf[1]->is_grabbed = false;
		fishes.pf[1]->in_cage = false;
		fishes.pf[2]->Px = -20.0;
		fishes.pf[2]->Py = 20.0;
		fishes.pf[2]->Pz = 0.0;
		fishes.pf[2]->yaw = 0.0; 
		fishes.pf[2]->roll = 0.0;
		fishes.pf[2]->pitch = 0.0;
		fishes.pf[2]->previous_x = fishes.pf[2]->Px;
		fishes.pf[2]->previous_y = fishes.pf[2]->Py;
		fishes.pf[2]->previous_z = fishes.pf[2]->Pz;
		fishes.pf[2]->is_grabbed = false;
		fishes.pf[2]->in_cage = false;
		pers1.Px = 20.0; 
		pers1.Py = 20.0; 
		pers1.Pz = 0.0; 
		pers1.pitch = 0.0; 
		pers1.roll = PI/2; 
		pers1.is_grabbed = false;
		pers1.in_cage = false;

		t0 = high_resolution_time();
		tp = 0.0;
		init = 1;
	}

	//Time Related Variables
	t = high_resolution_time() - t0;
	T = t - tp;	
	fps = 1 / T;
	tp = t;	

	//Static on-text screen
	text_xy("Place the fish inside the cage!", 10.0, 10.0, 20);
	text_xy("Commands: \nW-A-S-D-Q-E = Move Arms\nV = Toggle POV, G = Grab, R = Reset", 10.0, 1000.0, 10);
	text_xy("Bonus hint: You are an object!\nPress 1, 2 or 3 to auto-target", 1650.0, 1000.0, 10);
	text_xy("Fish =", 10.0, 90.0, 20);
	text_xy(score, 150.0, 90.0, 20);
	text_xy("Money = ", 10.0, 50.0, 20);
	text_xy(money, 150.0, 50.0, 20);
	text_xy("Press SPACEBAR to sell fish in cage!", 1400, 10.0, 20);
	
	//----------------------------------------------------------Keyboard Inputs--------------------------------------------------------
	if (KEY(VK_SPACE))											//KEY: SPACE
	{
		money += score * 5;
		score = 0;
		if (fishes.pf[1]->in_cage) fishes.pf[1]->delete_obj();
		if (fishes.pf[2]->in_cage) fishes.pf[2]->delete_obj();
		if (pers1.in_cage) pers1.delete_obj();
	}
	
	if (KEY(0x52))
	{
		init = 0;
	}

	//POV
	static bool third_POV = true;
	if (KEY(0x56))												//KEY: V
	{
		//Toggle View Point
		third_POV = !third_POV;									
		Sleep(200);
	}
	//Third Person View
	if (third_POV)												
	{	
		double eye_point[3 + 1] = { -1.0, 60.0, 60.0, 30.0 };
		double lookat_point[3 + 1] = { -1.0, 0.0, 0.0, 20.0 };		
		double up_dir[3 + 1] = { -1.0, 0.0, 0.0, 1.0 };				
		double fov = PI / 4;										
		set_view(eye_point, lookat_point, up_dir, fov);
	}
	//First Person View
	else
	{
		double eye_point[3 + 1] = { -1.0, pers1.Px, pers1.Py, pers1.Pz + 8.0 };
		double lookat_point[3 + 1] = { -1.0, end_effector.Px / 2, end_effector.Py / 2, end_effector.Pz / 2 };
		double up_dir[3 + 1] = { -1.0, 0.0, 0.0, 1.0 };				
		double fov = PI/2;										
		set_view(eye_point, lookat_point, up_dir, fov);
	}

	//Locate Objects
	static double ObjTheta[3 + 1];
	if (KEY(0x31))												//KEY: 1
	{
		locateObject(ObjTheta, *fishes.pf[1]);
		text_xy(end_effector.get_distance(*fishes.pf[1]), 1000.0, 300.0, 14);
		if (end_effector.get_distance(*fishes.pf[1]) < 3.0)		text_xy("(Press G!)", 1100.0, 300.0, 14);
	}
	else if (KEY(0x32))											//KEY: 2
	{
		locateObject(ObjTheta, *fishes.pf[2]);
		text_xy(end_effector.get_distance(*fishes.pf[2]), 1000.0, 300.0, 14);
		if (end_effector.get_distance(*fishes.pf[2]) < 3.0)		text_xy("(Press G!)", 1100.0, 300.0, 14);
	}
	else if (KEY(0x33))											//KEY: 3
	{
		locateObject(ObjTheta, pers1);
		text_xy(end_effector.get_distance(pers1), 1000.0, 300.0, 14);
		if (end_effector.get_distance(pers1) < 3.0)		text_xy("(Press G!)", 1100.0, 300.0, 14);
	}

	//								Some inputs were placed in "inputs" under the sim_step function
	//-------------------------------------------------------End Keyboard input-------------------------------------------------

	//Simulation
	dt = 0.00003;
	for (int i = 0; i < 300; i++)
	{
		sim_step(dt, arm1.yaw, arm2.pitch, arm3.pitch, ObjTheta);
		fishes.pf[1]->sim_roam(dt);
		fishes.pf[2]->sim_roam(dt);
		pers1.sim_fall(dt);
	}

	//Update Program
	fish_caught(cage, *fishes.pf[1], score);
	fish_caught(cage, *fishes.pf[2], score);
	fish_caught(cage, pers1, score);

	resolveCollision(*fishes.pf[1], *fishes.pf[2]);
	resolveCollision(*fishes.pf[2], pers1);
	resolveCollision(*fishes.pf[1], pers1);

	checkPickup(end_effector, *fishes.pf[1]);
	checkPickup(end_effector, *fishes.pf[2]);
	checkPickup(end_effector, pers1);

	//Draw all relevant objects
		//3DOF ROBOT
	arm1.draw();
	arm2.draw();
	arm3.draw();
	
		//OBJECTS
	fishes.draw();
	pers1.draw();
	cage.draw();
}

