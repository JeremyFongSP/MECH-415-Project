
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

// 3D graphics window size in pixels
int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 1500; // increase this to increase the window width
int HEIGHT_MAX = 1000; // increase this to increase the window height

// background colour for the scene
float BACK_R = 0.0f; // red colour component (0 to 1)
float BACK_G = 0.0f; // green colour component (0 to 1)
float BACK_B = 0.5f; // blue colour component (0 to 1)

// note the ratio of VMAX/VMIN should be less than 10,000 for most graphics cards.
double VMIN = 0.25; // units of m (or whatever units you draw your object in)
double VMAX = 1000.0; // units of m

using namespace std;

// note: the keyboard input macro is defined in "3D_graphics.h" file.

void draw_3D_graphics() //This function gets called repeatedly from the main(), based on the fastest fps possible. 

// note: the windows main function is inside the graphics
// library *.lib file (ie Vertices.lib).
{

	//Just testing things out. Subject to change.
	static double Px, Py, Pz,roll,pitch,yaw; //Variables
	static double Px2, Py2, Pz2, roll2, pitch2, yaw2; //Variables
	static double Px3, Py3, Pz3, roll3, pitch3, yaw3; //Variables

	
	static int init = 0; // initialization flag
	static double x=0.0,dx; // x - an offset for the object
	// z offset controlled by keyboard input
	static double z = 0.0,dz;
	static double t; // clock time from t0
	static double t0; // initial clock time

	static mesh m1("arm1.x"); // the mesh here gets constructed only once since  its a static local variable, so no need to put in the initialization section 
	static mesh m2("arm2.x");//You can keep adding max of 8 mesh files.
	static mesh m3("arm3.x");




	// initalization section
	//Only for non static local variables
	if(!init) {

		t0 = high_resolution_time(); // initial clock time (s)   =resolution_time() 

		init = 1;
	} 

	set_view(); // simple camera setting function
	//draw_XYZ(5.0);  // draws and setts axxes of 5m lenght( x:red, y: green, z:blue)
	t = high_resolution_time() - t0; //t  = time elapsed since start
	
	
	



	//CONTROL THE OBJECT
	//3 ways
	//Mechanisms: Through time(by adding t in Px,Py,Pz, or yaw pitch and roll) or by Keyboard
	//Control position(Px, Py, Pz)
	//Control the way it rolls on its own axis(yaw, pitch, roll)

	//For object m1
	Px = 0.0;
	Py = 0.0;
	Pz = 0.0;
	
	yaw = 3.14159/2;
	pitch = 0.0;
	roll = 3.14159/2;
	
	//For object m2
	Px2 = 0.0;
	Py2 = 0.0;
	Pz2 = 16.51*0.09;

	yaw2 =  yaw;
	pitch2 = 0.0;
	roll2 = 3.14159 / 2;

	//For object m3
	/*
	x, y and z coordinate of object m3 depends on the position of object m1 and m2
	*/
	Px3 =  0.0;
	Py3 = 0.0;
	Pz3 = 16.51*0.09;

	yaw3 = yaw;
	pitch3 = 0.0;
	roll3 = 3.14159/2;



	// move arm 3
	if (KEY(VK_UP) && KEY(VK_SPACE)) pitch3 += 3.14159/180;

	if (KEY(VK_DOWN) && KEY(VK_SPACE)) pitch3 -= 3.14159/180;

	// move arm 2
	if (KEY(VK_UP) && !KEY(VK_SPACE)) pitch2 += 3.14159/180;//increment position of z. Make sure there is Pz = ...+z

	if (KEY(VK_DOWN) && !KEY(VK_SPACE)) pitch2 -= 3.14159/180; //Decrement the position of z

	// move arm 1
	if (KEY(VK_RIGHT)) yaw += 3.14159/180;//increment position of X

	if (KEY(VK_LEFT)) yaw -= 3.14159/180; //Decrement the position of x.  Make sure Px= ...x

	
// draw x-file / mesh object
m1.Scale =0.09;
m1.draw(Px,Py,Pz,yaw,pitch,roll); 

m2.Scale = 0.09;
m2.draw(Px2, Py2, Pz2, yaw, pitch2, roll2); 


m3.Scale = 0.09;
m3.draw(Px3, Py3, Pz3, yaw, pitch3, roll3); 
	
}



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



// note: if you have an object with dynamic memory
// you can make it a static local variable
// and the constructor will only get called (along with 
// new) once