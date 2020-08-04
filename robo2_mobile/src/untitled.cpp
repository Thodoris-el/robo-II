//X = state vector
float x[5]; // =[x,y,theta,v,omega]
//initialize
x =[0.0,0.0,0.0,0.0,0.0];
float P[5];
P =[0.0,0.0,0.0,0.0,0.0];
sigma_omega =  0.002;
sigma_length = 0.01;
sigma_acc = 0.002;
sigma_theta = 0.002;

//measurement convariance;

float R[8][8] = {{pow(sigma_length,2),0,0,0,0,0,0,0},
{0,pow(sigma_length,2),0,0,0,0,0,0},
{0,0,pow(sigma_length,2),0,0,0,0,0},
{0,0,0,pow(sigma_length,2),0,0,0,0},
{0,0,0,0,pow(sigma_length,2),0,0,0},
{0,0,0,0,0,pow(sigma_length,2),0,0},
{0,0,0,0,0,0,pow(sigma_length,2),0},
{0,0,0,0,0,0,0,pow(sigma_length,2)}};

float time = 0;

