// class for the arms with the position and have a pointer to a mesh


class simulation {
	static double tp;											// previous time
	static double t0;											// initial clock time
	static double t;

public:
	int init();
	simulation(char *mesh_name, double position[3], double rotation[3]);
	~simulation();
	int deinit();
	void draw_sim(double position[3], double rotation[3]);
	

};

void key_input();