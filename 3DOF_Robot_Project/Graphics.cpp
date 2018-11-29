// graphics.cpp will allow us to draw the mesh of the simulated robot. and move according to the input keys that the users will do.
// with that, the robot will also move using the inputs and the equations.




#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions
#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0
#include <iomanip>   // I/O manipulators
#include <windows.h> // for keyboard input

#include "Timer.h" // for measuring time
#include "Graphics.h"

int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 1500; // increase this to increase the window width
int HEIGHT_MAX = 1500; // increase this to increase the window height

float BACK_R = 0.0f; // red colour component (0 to 1)
float BACK_G = 0.0f; // green colour component (0 to 1)
float BACK_B = 0.5f; // blue colour component (0 to 1)

double VMIN = 0.25; // units of m (or whatever units you draw your object in)
double VMAX = 1000.0; // units of m

using namespace std;

void draw_3DOF_arms()
{
	static double Px, Py, Pz, roll, pitch, yaw;
	static int init = 0; // initialization flag

	static double x = 0.0, dx; // x - an offset for the object
	// NOTE: it almost never hurts and can avoid errors if 
	// you make all your local variables static in draw_3D_graphics() 

	// z offset controlled by keyboard input
	static double z = 0.0, dz;

	static double t; // clock time from t0
	static double t0; // initial clock time

	// declare/load x-file mesh object to be drawn later
	// note: the maximum number of meshes/x-files is eight
	// note: x-file drawings should be centered at the origin when drawn
	static mesh m1("tiger.x"); // the mesh here gets constructed only once since it
	// its a static local variable, so no need to put in the initialization section 

	//static mesh m2("plane.x");

	// initalization section
	if (!init) {

		// anything which is slow and done once should be put here
		// especially file opening

		// put your initialization tasks here
		// such as opening files, allocating dynamic memory
		// time consuming initialization calculations, etc.

		// note: if you have an object with dynamic memory
		// you can make it a static local variable
		// and the constructor will only get called (along with 
		// new) once

		t0 = high_resolution_timer(); // initial clock time (s)

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
	//draw_XYZ(5.0);  // set axes of 5 m length

	// read clock time (resolution is 0.1 microseconds)
	// this function is useful for real-time / virtual reality simulations
	// (ie where simulation time = actual/clock time)
	// note: don't use this as t in your simulation programs yet
	// I will show you how to incorporate the clock with your simulations
	// note: the upper value value of the time function where
	// it rolls is very very large
	t = high_resolution_timer() - t0; // time since the program started (s)

	// draw a mesh object 
	// these variables can be changed dynamically to animate
	// (translate and rotate) the object

	Px = z;
	Py = 0.0;
	Pz = 0.5 + sin(5 * t);
	yaw = 3.14159 / 2 + t;
	pitch = 0.0;
	roll = 3.14159 / 2;

	//m1.Scale = 1.5 + sin(t);

	// draw x-file / mesh object
	m1.draw(Px, Py, Pz, yaw, pitch, roll);

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

	// read keyboard input using KEY macro

	// note: pressing the escape key closes the graphics window

	dz = 0.003; // z offset increment

	if (KEY(VK_UP)) z += dz;

	if (KEY(VK_DOWN)) z -= dz;

	// alternative up using a character key
	if (KEY('U')) z += dz;
}