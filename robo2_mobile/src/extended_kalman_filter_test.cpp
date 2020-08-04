/*
px = position x
py = position y
v = velocity (vx, vy)
a = accelaration
w = rad velocity
th = thetan

State 
px = px + v*cos(th)*dt + 1/2 *cos(th)*a*dt^2
py = py + v*sin(th)*dt + 1/2 *sin(th)*a*dt^2
v = v + a*dt
th = th + w*dt


Ak = [[1,0,-v*sin(th)*dt - 1/2 * a*sin(th)*dt^2, cos(th)],
	[0,1,-v*cos(th)*dt - 1/2 * a*cos(th)*dt^2, sin(th)],
	[0,0,1,0],
	[0,0,0,1]]

Bk = [[1/2 * cos(th)*dt^2,0],
	[1/2 * sin(th)*dt^2,0],
	[0,dt],
	[dt,0]]1/2 * cos(th) *dt^2 * da

wk = [[1/2 * cos(th) *dt^2 * da],
	[1/2 * sin(th) *dt^2 * da],
	[dt * dw],
	[dt*da]]


Cw = [[1/4 *cos^2(th)*dt^4*da*2, 1/4 *cos(th)*sin(th)*dt^4*da*2, 0, 1/2 *cos(th)*dt^3*da^2],
	[1/4 *cos(th)*sin(th)*dt^4*da*2, 1/4 *sin^2(th)*dt^4*da*2, 0, 1/2 *sin(th)*dt^3*da^2],
	[0,0,dt^2*dw^2,0],
	[1/2 * cos(th)*dt^3*da^2, 1/2 *sin(th)*dt^3*da^2,0,dt^2*da^2]]


 
sensor_F = cos(or) * px + sin(or) * py + 0 * v
sensor_R = sin(or) * px + cos(or) *py + 0 * v
sensor_L = sin(or) * px + cos(or) *py + 0 * v (?)
sensor_FR = cos(or+45) * px + sin(or+45) + 0 * v
sensor_FL = cos(or+45) * px + sin(or+45) + 0 * v

sensor_H = [[cos(or),sin(or),0,0],
			[sin(or),cos(or),0,0],
			[sin(or),cos(or),0,0],
			[cos(or+45),sin(or+45),0,0],
			[cos(or+45),sin(or+45),0,0]]

z =[[x],
	[y],
	[th]

*/



//sensor variables

float sonarF,sonarFL,sonarFR,sonarL,sonarR;



#read sensors
ros::Subscriber sonarFront_sub = n.subscribe("/sensor/sonar_F", 1, &sonarFrontCallback);
  ros::Subscriber sonarFrontLeft_sub = n.subscribe("/sensor/sonar_FL", 1, &sonarFrontLeftCallback);
  ros::Subscriber sonarFrontRight_sub = n.subscribe("/sensor/sonar_FR", 1, &sonarFrontRightCallback);
  ros::Subscriber sonarLeft_sub = n.subscribe("/sensor/sonar_L", 1, &sonarLeftCallback);
  ros::Subscriber sonarRight_sub = n.subscribe("/sensor/sonar_R", 1, &sonarRightCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu", 1, &imuCallback);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/mymobibot/cmd_vel", 1);
  ros::Subscriber imu_sub = n.subscribe("/imu", 1, &imuCallback);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/mymobibot/cmd_vel", 1);


// eroors

float dl, da,dw,dth;

dl = 0.01; //m
da = 0.002; //m/s^2
dth = 0.002; //rad
dw = 0.002; //rad/s

float L1,L2,L3,L4;
L1 = 0.018;//m
L2 = 0.05;//m
L3 = 0.1;//m
L4 = 0.2;//m

float Cv[6][6];
for(int i = 0;i<6;i++){
	for(int j = 0; j<6){
		if(i==j && i !=5){
			Cv[i][j] = dl*dl;
		}
		else if(i==j){
			Cv[i][j] = dth*dth;
		}
		else{
			Cv[i][j] = 0;
		}
	}
}


float Cw[4][4];
Cw[0][0] = 1/4 *cos(th)*cos(th)*pow(dt,4)*da*da;
Cw[0][1] = 1/4 *cos(th)*sin(th)*pow(dt,4*da*da;
Cw[0][2] = 0;
Cw[0][3] = 1/2 *cos(th)*pow(dt,3)*da*da;

Cw[1][0] = 1/4 *sin(th)*cos(th)*pow(dt,4)*da*da;
Cw[1][1] = 1/4 *sin(th)*sin(th)*pow(dt,4*da*da;
Cw[1][2] = 0;
Cw[1][3] = 1/2 *sin(th)*pow(dt,3)*da*da;

Cw[2][0] = 0;
Cw[2][1] = 0;
Cw[2][2] = dt*dt*dw*dw;
Cw[2][3] = 0;

Cw[3][0] = 1/2 * cos(th)*pow(dt,3)*da*da;
Cw[3][1] = 1/2 *sin(th)*pow(dt,3)*da*da;
Cw[3][2] = 0;
Cw[3][3] = dt*dt*da*da;



float tmplength;
tmplength = (L4/2 - L1)*(L4/2 - L1) + (L4 - L1)*(L4 - L1);
tmplength = sqrt(tmplength);
float z[5][4];
z[0][0] = (L4+lfron)*cos(th);
z[0][1] = (L4+lfron)*sin(th);
z[0][2] = 0;
z[0][3] = 0;

z[1][0] = (tmpLength+lfrontl)*cos(th+45);
z[1][1] = (tmpLength+lfrontl)*sin(th+45);
z[1][2] = 0;
z[1][3] = 0;

z[2][0] = (lfrontr)*cos(th+45);
z[2][1] = (lfrontr)*sin(th+45);
z[2][2] = 0;
z[2][3] = 0;

z[3][0] = (L2+L4/2+lleft)*sin(th);
z[3][1] = (L2+L4/2+lleft)*cos(th);
z[3][2] = 0;
z[3][3] = 0;

z[3][0] = (L2+L4/2+lright)*sin(th);
z[3][1] = (L2+L4/2+lright)*cos(th);
z[3][2] = 0;
z[3][3] = 0;

