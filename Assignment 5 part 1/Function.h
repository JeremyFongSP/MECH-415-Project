// unpacking of the parameters and packing of the input force
void calculate_inputs(const double X[], double t, int N, double U[], int M);

// calculates the derivative of the state variables for the project
void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[], double invM[4][4], double c[4]);