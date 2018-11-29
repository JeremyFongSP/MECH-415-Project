// graphics.h have to screate the headers of the functions that graphics.cpp will use.


class mesh
{
	int mesh_number;

public:
	double roll_0, yaw_0, pitch_0;
	double theta1_0, theta2_0, theta3_0;
	mesh(char file_name[]);
	void draw(double Tx, double Ty, double Tz, double yaw, double pitch, double roll);
};

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

void draw_3DOF_arms();

void set_view();

void set_view(double *eye_point, double *lookat_point, double *up_dir, double fov = 3.14159 / 4);
// sets the view and perspective matrices
// eye_point[1] - x component of eye location
// eye_point[2] - y component of eye location
// eye_point[3] - z component of eye location
// lookat_point[1] - x component of look at point
// lookat_point[2] - y component of look at point
// lookat_point[3] - z component of look at point
// up_dir[1] - x component of up direction
// up_dir[2] - y component of up direction
// up_dir[3] - z component of up direction
// fov - field of view (45 deg default)

void set_light(int light_number, double dir_x, double dir_y, double dir_z,
	double R, double G, double B, int light_switch);
// Sets a directional light for the scene
// Note that many lights may be active at a time 
// (but each one slows down the rendering of our scene)
// light_number - number of the light, calling this function with the same number will
// overwrite the current setting
// dir_x - x component of the light direction
// dir_y - y component of the light direction
// dir_z - z component of the light direction
// R - Red colour components (0<=R<=1) of the light
// G - Green colour components (0<=G<=1) of the light
// B - Blue colour components (0<=B<=1) of the light
// light_switch - set equal to zero to turn off the light, non-zero to turn it on

void text_xy(char *str, double x, double y, int size = 14);
// place 2D text at pixel location (x,y)
// str - text string
// x - x pixel location of text
// y - y pixel location of text
// size = font size of the text (default = 14)