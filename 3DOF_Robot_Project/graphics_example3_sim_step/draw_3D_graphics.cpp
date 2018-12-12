
// example of how to use x-files and meshes

#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0
#include <iomanip>   // I/O manipulators

#include <windows.h> // for keyboard input

// user defined functions
#include "timer.h" // for measuring time
#include "rotation.h" // for computing rotation functions
#include "3D_graphics.h" // for DirectX 3D graphics

#include "my_project.h"

const double PI = atan(1) * 4;

// 3D graphics window size in pixels
int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 1500; // increase this to increase the window width
int HEIGHT_MAX = 1000; // increase this to increase the window height

// background colour for the scene
float BACK_R = 0.0f; // red colour component (0 to 1)
float BACK_G = 0.0f; // green colour component (0 to 1)
float BACK_B = 0.5f; // blue colour component (0 to 1)

// default min and max viewing distances.
// objects closer than VMIN and farther than VMAX are not drawn (ie cannot be seen).
// note the ratio of VMAX/VMIN should be less than 10,000 for most graphics cards.
double VMIN = 0.25; // units of m (or whatever units you draw your object in)
double VMAX = 1000.0; // units of m

// global variable for keyboard input
extern unsigned char Key_input[256];

using namespace std;

// define a global file output stream after namespace
ofstream fout("debug.csv");

void draw_3D_graphics() // draw one frame of the scene
// draw DirectX 3D graphics
// note: this function gets called repeatedly from the windows main function
// at the fastest frame rate (fps) possible.
// one time initializations should be placed in the initialization sections
{
	static double Px, Py, Pz, roll, pitch, yaw;								//Variables
	static double Px2, Py2, Pz2, roll2, pitch2 = PI / 12 - PI / 6, yaw2;	//Variables
	static double Px3, Py3, Pz3, roll3, pitch3 = PI/12, yaw3;				//Variables

	static int init = 0; // initialization flag

	static double x=0.0,dx; // x - an offset for the object
	// NOTE: it almost never hurts and can avoid errors if 
	// you make all your local variables static in draw_3D_graphics() 

	// z offset controlled by keyboard input
	static double z = 0.0,dz;

	static double t; // clock time from t0
	static double t0; // initial clock time

	static double T, fps, tp, dt;


	static mesh m1("arm1_test.x"); // the mesh here gets constructed only once since  its a static local variable, so no need to put in the initialization section 
	static mesh m2("arm2.x");		//You can keep adding max of 8 mesh files.
	static mesh m3("arm2.x");

	// initalization section
	if(!init) {

		// put your initialization tasks here
		// such as opening files, allocating dynamic memory
		// time consuming initialization calculations, etc.

		// note: if you have an object with dynamic memory
		// you can make it a static local variable
		// and the constructor will only get called (along with 
		// new) once

		t0 = high_resolution_time(); // initial clock time (s)

		//CONTROL THE OBJECT
		//3 ways
		//Mechanisms: Through time(by adding t in Px,Py,Pz, or yaw pitch and roll) or by Keyboard
		//Control position(Px, Py, Pz)
		//Control the way it rolls on its own axis(yaw, pitch, roll)

		tp = 0.0; // previous time

		init = 1;
	} // end of initialization section

 	// set the view point with the keyboard
	// (it is looking at the origin)
	// q increases X, a decreases X
	// w increases Y, s decreases Y	
	// e increases Z, d decreases Z	
	// r increases diagonally, f decreases diagonally
	set_view(); // simple camera setting function -- later on remove it

	// 2D orthogonal view in x-y coord 
	// the argument is the width and height of the graphics window
	// don't run this the same time as set_view()
//	set_2D_view(5.0,5.0); // comment out the above if you use this

	// draw the axes (red = x, y = green, z = blue)
	draw_XYZ(5.0);  // set axes of 5 m length

	// read clock time (resolution is 0.1 microseconds)
	// this function is useful for real-time / virtual reality simulations
	// (ie where simulation time = actual/clock time)
	// note: don't use this as t in your simulation programs yet
	// I will show you how to incorporate the clock with your simulations
	// note: the upper value value of the time function where
	// it rolls is very very large
	t = high_resolution_time() - t0; // time since the program started (s)

	T = t - tp; // calculate dt frame or period

	fps = 1 / T;

	tp = t; // save previous time for later
	
	fout << t << "," << T << "," << fps << "\n";

	// for a simulation with 3D graphics you would simulate
	// your system here, perhaps by doing one or more Euler simulation steps.

	// fast but unstable
//	dt = 0.01; // for small simulation error
//	sim_step(dt,y);
	
	// note: a better way to speed the simulation
	// up is to increase the number of simulation steps
	// for each graphics function call, eg
	dt = 0.001;
//	for (int i = 0; i < 100; i++) sim_step(dt, yaw, pitch2, pitch3); // fast and stable					<<NEED THIS

	// this assumes the sim_step exexution time
	// is much smaller than the graphics loop time

	// if you want to have multiple sim steps per graphics draw
	// allows the reduction of the simulation error
	// by reducing dt
	// use 1000 for a very fast tiger
//	for (i=0; i < 1000;i++) sim_step(dt,y);

	// connect graphics parameters with the simulation output
	//For object m1
	//Px = 2.0 + x;
	Px = 0.0;
	Py = 0.0;
	Pz = 0.0;
//	yaw = 0.0;
	pitch = 0.0;
	roll = PI / 2;

	//For object m2
	Px2 = 0.0;
	Py2 = 0.0;
	Pz2 = 16.5;						//L base ~ 16.5
	yaw2 =  yaw;
//	pitch2 = PI/12 - PI/6;			//neutral = PI/12; negative is upwards
	roll2 = 0.0;

	//For object m3
	Px3 = 17.5*cos(pitch2)*cos(yaw);
	Py3 = 17.5*cos(pitch2)*sin(yaw);
	Pz3 = 17.5 - 17.5*sin(pitch2 - PI/12);		//L Arm ~ 17.5; negative is upwards again due to pitch; remove neutral
	yaw3 = yaw;
//	pitch3 = PI/12;
	roll3 = 0.0;

	//Keyboard IN
	if (KEY(VK_UP)) pitch3 -= PI*dt / 2;			//increment position of z. Make sure there is Pz = ...+z
	if (KEY(VK_DOWN)) pitch3 += PI*dt / 2;			//Decrement the position of z
	if (KEY(VK_RIGHT)) yaw += PI*dt/2;				//increment position of X
	if (KEY(VK_LEFT)) yaw -= PI*dt / 2;				//Decrement the position of x.  Make sure Px= ...x
	if (KEY(0x31)) pitch2 += PI*dt / 2;				//KEY: 1
	if (KEY(0x32)) pitch2 -= PI*dt / 2;				//KEY: 2
	
	// draw x-file / mesh object
//	m1.Scale = 0.1;
	m1.draw(Px,Py,Pz,yaw,pitch,roll); 

//	m2.Scale = 0.1;
	m2.draw(Px2, Py2, Pz2, yaw2, pitch2, roll2); 

//	m3.Scale = 0.1;
	m3.draw(Px3, Py3, Pz3, yaw, pitch3, roll3); 

	dz = 0.003; // z offset increment
	dx = 0.003;//x offset increment
	

	// note: for your project
	// you would call the simulate_step and draw
	// functions from your class here
	// -- your class would be a static local variable
	// of the draw_3D_graphics() function (like the mesh objects)

	// without using classes you would call your calculate_xd 
	// funtion, perform one Euler simulation step for dt, and 
	// then use some of the state variables for translations and 
	// rotation in your mesh draw functions.
	// the state vector, derivative, and simulation time
	// would be static local variables of draw_3D_graphics()

	// illustration of keyboard input

	// note: pressing the escape key closes the graphics window
	// note: you could put the key input code in a seperate
	// function as long as it has access to the external global Key_input
/*
	dz = 0.1; // z offset increment

	if( Key_input[VK_UP] ) { // pressing the up key
		Key_input[VK_UP] = 0; // need to have this part
		z += dz; // adjust the offset each time the key is pressed
	}

	if( Key_input[VK_DOWN] ) {
		Key_input[VK_DOWN] = 0;
		z -= dz;
	}
*/
}

