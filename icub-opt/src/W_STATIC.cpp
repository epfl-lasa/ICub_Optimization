#include "iCubOpt.hpp"
using namespace std; 

iCubOpt * W_STATIC_M;

Cvector pos_new(38);
Cvector vel_new(38);

Cvector vel(38) ;
Cvector pos(38) ;

void obj(int    *Status, int *n,    double sx[], 
		int    *needF,  int *neF,  double sF[],
		int    *needG,  int *neG,  double sG[],
		char      *cu,  int *lencu,
		int    iu[],    int *leniu,
		double ru[],    int *lenru )
{ 
	W_STATIC_M->W_STATIC_shot(sx, sF, sG);
} 

void iCubOpt::W_STATIC_delete()
{  
	snopt_delete_space(W_STATIC_vars);
	delete W_STATIC_vars.snopt;
} 


// This function is implemented ONLY in the first step of optimization, so its initial condition is defined by user.
void iCubOpt::W_STATIC_init()
{ 
	W_STATIC_vars.num_var = 38;   // number of optimization variables
	// total number of equations for cost function and constraints. Note: constrains on opt. variables are not included here.
	W_STATIC_vars.num_eq = 2 ; //80; //39;//77;   
	W_STATIC_vars.num_deriv = 54 ; //281;//168; //54 ;//92;  //number of non-zero elements of Gradient Matrix
	W_STATIC_vars.ObjRow  = 0;    
	W_STATIC_vars.ObjAdd  = 0; //

	W_STATIC_vars.snopt = new snoptProblemA("W_STATIC","");
	W_STATIC_M = this;

	W_STATIC_vars.snopt->setUserFun(obj);

	snopt_allocate_space(W_STATIC_vars);

	// set the initial condition, upper and lower band for optimization variable (joint acceleration)
	// Note: Upper and lower band for joints acceleration should be modified. 
	
	// Note: First six variables are related to six DOF of root link. 
	// Note: Here, it's assumed that root link is fixed. So, upper and lower band for it's acceleration (and velocity) are close to zero.
	W_STATIC_vars.xlow[0]= -1.000000e-08; W_STATIC_vars.x0[0]= 0.000000; W_STATIC_vars.xupp[0]= 1.000000e-08; W_STATIC_vars.xstate[0]= 0;
	W_STATIC_vars.xlow[1]= -1.000000e-08; W_STATIC_vars.x0[1]= 0.000000; W_STATIC_vars.xupp[1]= 1.000000e-08; W_STATIC_vars.xstate[1]= 0;
	W_STATIC_vars.xlow[2]= -1.000000e-08; W_STATIC_vars.x0[2]= 0.000000; W_STATIC_vars.xupp[2]= 1.000000e-08; W_STATIC_vars.xstate[2]= 0;
	W_STATIC_vars.xlow[3]= -1.000000e-08; W_STATIC_vars.x0[3]= 0.000000; W_STATIC_vars.xupp[3]= 1.000000e-08; W_STATIC_vars.xstate[3]= 0;
	W_STATIC_vars.xlow[4]= -1.000000e-08; W_STATIC_vars.x0[4]= 0.000000; W_STATIC_vars.xupp[4]= 1.000000e-08; W_STATIC_vars.xstate[4]= 0;
	W_STATIC_vars.xlow[5]= -1.000000e-08; W_STATIC_vars.x0[5]= 0.000000; W_STATIC_vars.xupp[5]= 1.000000e-08; W_STATIC_vars.xstate[5]= 0;

	/// torso

	W_STATIC_vars.xlow[6]= -24;	 W_STATIC_vars.x0[6]= 0.000000; W_STATIC_vars.xupp[6]= 24;    W_STATIC_vars.xstate[6]= 0;
	W_STATIC_vars.xlow[7]= -24;  	 W_STATIC_vars.x0[7]= 0.000000; W_STATIC_vars.xupp[7]= 24;    W_STATIC_vars.xstate[7]= 0;
	W_STATIC_vars.xlow[8]= -24;  	 W_STATIC_vars.x0[8]= 0.000000; W_STATIC_vars.xupp[8]= 24;    W_STATIC_vars.xstate[8]= 0;

	/// left body

	W_STATIC_vars.xlow[9]=  -12 ;    W_STATIC_vars.x0[9]= 0.000000;   W_STATIC_vars.xupp[9]= 12; 	W_STATIC_vars.xstate[9]= 0;
	W_STATIC_vars.xlow[10]= -12 ;    W_STATIC_vars.x0[10]= 0.000000;  W_STATIC_vars.xupp[10]= 12;   W_STATIC_vars.xstate[10]= 0;
	W_STATIC_vars.xlow[11]= -12 ;    W_STATIC_vars.x0[11]= 0.000000;  W_STATIC_vars.xupp[11]= 12;   W_STATIC_vars.xstate[11]= 0;

	W_STATIC_vars.xlow[12]= -12 ;    W_STATIC_vars.x0[12]= 5.00000; W_STATIC_vars.xupp[12]= 12;    W_STATIC_vars.xstate[12]= 0;

	W_STATIC_vars.xlow[13]= -0.45;   W_STATIC_vars.x0[13]= 0.000000; W_STATIC_vars.xupp[13]= 0.45;  W_STATIC_vars.xstate[13]= 0;
	W_STATIC_vars.xlow[14]= -0.65;   W_STATIC_vars.x0[14]= 0.000000; W_STATIC_vars.xupp[14]= 0.65;  W_STATIC_vars.xstate[14]= 0;
	W_STATIC_vars.xlow[15]= -0.65;   W_STATIC_vars.x0[15]= 0.000000; W_STATIC_vars.xupp[15]= 0.65;  W_STATIC_vars.xstate[15]= 0;

	W_STATIC_vars.xlow[16]= -40;     W_STATIC_vars.x0[16]= 0.000000; W_STATIC_vars.xupp[16]= 40;    W_STATIC_vars.xstate[16]= 0;
	W_STATIC_vars.xlow[17]= -40; 	 W_STATIC_vars.x0[17]= 0.000000; W_STATIC_vars.xupp[17]= 40;    W_STATIC_vars.xstate[17]= 0;
	W_STATIC_vars.xlow[18]= -40;	 W_STATIC_vars.x0[18]= 0.000000; W_STATIC_vars.xupp[18]= 40;    W_STATIC_vars.xstate[18]= 0;

	W_STATIC_vars.xlow[19]= -40;     W_STATIC_vars.x0[19]= 0.000000; W_STATIC_vars.xupp[19]= 40;     W_STATIC_vars.xstate[19]= 0;
	W_STATIC_vars.xlow[20]= -40;     W_STATIC_vars.x0[20]= 0.000000; W_STATIC_vars.xupp[20]= 40;     W_STATIC_vars.xstate[20]= 0;
	W_STATIC_vars.xlow[21]= -40;     W_STATIC_vars.x0[21]= 0.000000; W_STATIC_vars.xupp[21]= 40;     W_STATIC_vars.xstate[21]= 0;

	//// right body

	W_STATIC_vars.xlow[22]= -12;      W_STATIC_vars.x0[22]= 0.000000;  W_STATIC_vars.xupp[22]= 12; W_STATIC_vars.xstate[22]= 0;
	W_STATIC_vars.xlow[23]= -12 ;     W_STATIC_vars.x0[23]= 0.000000;  W_STATIC_vars.xupp[23]= 12;   W_STATIC_vars.xstate[23]= 0;
	W_STATIC_vars.xlow[24]= -12 ;     W_STATIC_vars.x0[24]= 0.000000;  W_STATIC_vars.xupp[24]= 12;   W_STATIC_vars.xstate[24]= 0;

	W_STATIC_vars.xlow[25]= -12 ;     W_STATIC_vars.x0[25]= 0.000000; W_STATIC_vars.xupp[25]= 12;    W_STATIC_vars.xstate[25]= 0;

	W_STATIC_vars.xlow[26]= -0.45;    W_STATIC_vars.x0[26]= 0.000000; W_STATIC_vars.xupp[26]= 0.45;  W_STATIC_vars.xstate[26]= 0;
	W_STATIC_vars.xlow[27]= -0.65;    W_STATIC_vars.x0[27]= 0.000000; W_STATIC_vars.xupp[27]= 0.65;  W_STATIC_vars.xstate[27]= 0;
	W_STATIC_vars.xlow[28]= -0.65;    W_STATIC_vars.x0[28]= 0.000000; W_STATIC_vars.xupp[28]= 0.65;  W_STATIC_vars.xstate[28]= 0;

	W_STATIC_vars.xlow[29]= -40;      W_STATIC_vars.x0[29]= 0.000000; W_STATIC_vars.xupp[29]= 40;    W_STATIC_vars.xstate[29]= 0;
	W_STATIC_vars.xlow[30]= -40; 	  W_STATIC_vars.x0[30]= 0.000000; W_STATIC_vars.xupp[30]= 40;    W_STATIC_vars.xstate[30]= 0;
	W_STATIC_vars.xlow[31]= -40;	  W_STATIC_vars.x0[31]= 0.000000; W_STATIC_vars.xupp[31]= 40;    W_STATIC_vars.xstate[31]= 0;

	W_STATIC_vars.xlow[32]= -40;      W_STATIC_vars.x0[32]= 0.000000; W_STATIC_vars.xupp[32]= 40;     W_STATIC_vars.xstate[32]= 0;
	W_STATIC_vars.xlow[33]= -40;      W_STATIC_vars.x0[33]= 0.000000; W_STATIC_vars.xupp[33]= 40;     W_STATIC_vars.xstate[33]= 0;
	W_STATIC_vars.xlow[34]= -40;      W_STATIC_vars.x0[34]= 0.000000; W_STATIC_vars.xupp[34]= 40;     W_STATIC_vars.xstate[34]= 0;

	/// head

	W_STATIC_vars.xlow[35]= -20;     W_STATIC_vars.x0[35]= 0.000000; W_STATIC_vars.xupp[35]= 20;     W_STATIC_vars.xstate[35]= 0;
	W_STATIC_vars.xlow[36]= -20;     W_STATIC_vars.x0[36]= 0.000000; W_STATIC_vars.xupp[36]= 20;     W_STATIC_vars.xstate[36]= 0;
	W_STATIC_vars.xlow[37]= -20;     W_STATIC_vars.x0[37]= 0.000000; W_STATIC_vars.xupp[37]= 20;     W_STATIC_vars.xstate[37]= 0;


	/////////////////////////////////
	// F(0) is the equation of cost function (left end-effector velocity)

	W_STATIC_vars.Flow[0]= -1.000000e+20; W_STATIC_vars.Fupp[0]= 1.000000e+20;  

	// Equation F(1) to F(38) are related to constraints on joints angles. Upper and lower magnitude of joint angles are written based on iCub SDF file. 

	/*	
	W_STATIC_vars.Flow[1]= -1.000000e+20; W_STATIC_vars.Fupp[1]= 1.000000e+20;  // root link
	W_STATIC_vars.Flow[2]= -1.000000e+20; W_STATIC_vars.Fupp[2]= 1.000000e+20;
	W_STATIC_vars.Flow[3]= -1.000000e+20; W_STATIC_vars.Fupp[3]= 1.000000e+20;
	W_STATIC_vars.Flow[4]= -1.000000e+20; W_STATIC_vars.Fupp[4]= 1.000000e+20;
	W_STATIC_vars.Flow[5]= -1.000000e+20; W_STATIC_vars.Fupp[5]= 1.000000e+20;
	W_STATIC_vars.Flow[6]= -1.000000e+20; W_STATIC_vars.Fupp[6]= 1.000000e+20;

	W_STATIC_vars.Flow[7]= -0.383972; W_STATIC_vars.Fupp[7]= 1.46608;            // torso
	W_STATIC_vars.Flow[8]= -0.680678; W_STATIC_vars.Fupp[8]= 0.680678;
	W_STATIC_vars.Flow[9]= -1.02974; W_STATIC_vars.Fupp[9]= 1.02974;

	W_STATIC_vars.Flow[10]= -1.65806; W_STATIC_vars.Fupp[10]= 0.0872665;           // left shoulder
	W_STATIC_vars.Flow[11]= 0.00000; W_STATIC_vars.Fupp[11]= 2.80649;
	W_STATIC_vars.Flow[12]= -0.645772; W_STATIC_vars.Fupp[12]= 1.74533;

	W_STATIC_vars.Flow[13]= 0.0959931; W_STATIC_vars.Fupp[13]= 1.85005;          // left elbow

	W_STATIC_vars.Flow[14]= -0.872665; W_STATIC_vars.Fupp[14]= 0.872665;         // left wrist
	W_STATIC_vars.Flow[15]= -1.13446; W_STATIC_vars.Fupp[15]= 0.174533;
	W_STATIC_vars.Flow[16]= -0.436332; W_STATIC_vars.Fupp[16]= 0.436332;

	W_STATIC_vars.Flow[17]= -0.767945; W_STATIC_vars.Fupp[17]= 2.30383;         // left hip
	W_STATIC_vars.Flow[18]= -0.296706; W_STATIC_vars.Fupp[18]= 2.07694;
	W_STATIC_vars.Flow[19]= -1.37881; W_STATIC_vars.Fupp[19]= 1.37881;

	W_STATIC_vars.Flow[20]= -2.18166; W_STATIC_vars.Fupp[20]= 0.401426;         // left foot
	W_STATIC_vars.Flow[21]= -0.733038; W_STATIC_vars.Fupp[21]= 0.366519;
	W_STATIC_vars.Flow[22]= -0.418879; W_STATIC_vars.Fupp[22]= 0.418879;

	W_STATIC_vars.Flow[23]= -1.65806; W_STATIC_vars.Fupp[23]= 0.0872665;        // right shoulder
	W_STATIC_vars.Flow[24]= 0.00000; W_STATIC_vars.Fupp[24]= 2.80649;
	W_STATIC_vars.Flow[25]= -0.645772; W_STATIC_vars.Fupp[25]= 1.74533;

	W_STATIC_vars.Flow[26]= 0.0959931; W_STATIC_vars.Fupp[26]= 1.85005;          // right elbow

	W_STATIC_vars.Flow[27]= -0.872665; W_STATIC_vars.Fupp[27]= 0.872665;         // right wrist
	W_STATIC_vars.Flow[28]= -1.13446; W_STATIC_vars.Fupp[28]= 0.174533;
	W_STATIC_vars.Flow[29]= -0.436332; W_STATIC_vars.Fupp[29]= 0.436332;

	W_STATIC_vars.Flow[30]= -0.767945; W_STATIC_vars.Fupp[30]= 2.30383;         // right hip
	W_STATIC_vars.Flow[31]= -0.296706; W_STATIC_vars.Fupp[31]= 2.07694;
	W_STATIC_vars.Flow[32]= -1.37881; W_STATIC_vars.Fupp[32]= 1.37881;

	W_STATIC_vars.Flow[33]= -2.18166; W_STATIC_vars.Fupp[33]= 0.401426;         // right foot
	W_STATIC_vars.Flow[34]= -0.733038; W_STATIC_vars.Fupp[34]= 0.366519;
	W_STATIC_vars.Flow[35]= -0.418879; W_STATIC_vars.Fupp[35]= 0.418879;

	W_STATIC_vars.Flow[36]= -0.523599; W_STATIC_vars.Fupp[36]= 0.383972;        // neck
	W_STATIC_vars.Flow[37]= -0.349066; W_STATIC_vars.Fupp[37]= 0.349066;
	W_STATIC_vars.Flow[38]= -0.767945; W_STATIC_vars.Fupp[38]= 0.767945;

	// Equation F(39) to F(76) are related to constraints on joints velocity. 

	for (unsigned int i = 39; i < 45; i++ )
	{
		W_STATIC_vars.Flow[i]= -1.000000e-08;
		W_STATIC_vars.Fupp[i]= 1.000000e-08;
	}

	for (unsigned int i = 45; i < 77; i++ )
	{
		W_STATIC_vars.Flow[i]= -0.785;
		W_STATIC_vars.Fupp[i]= 0.785;
	}
	
	// Equations F(77) and F(78) are related to x and y position of center of mass
	
	W_STATIC_vars.Flow[77]= -5.000000e-02; W_STATIC_vars.Fupp[77]= 15.000000e-02;
	W_STATIC_vars.Flow[78]= -7.000000e-02; W_STATIC_vars.Fupp[78]= 7.000000e-02;
	
	*/

	// In order to see the effect of this constraint, we removed all other constraint.
	// To solve the general problem, uncomment all above bands (from F(1) to F(78)) and modify the index of the following band:
	//  Equation F(79) is related to direction of velocity in the next step

	W_STATIC_vars.Flow[1]= -2.000000e-01; W_STATIC_vars.Fupp[1]= 2.000000e-01;
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Patern of non-zero elements of gradient matrxi should be defined. Since the velocity of left end-effector is function of left upper body,.. 
	// so, the first row of gradient matrix has 16 non-zero elements. (root link(6)+torso(3)+left shoulder(3)+left elbow(1)+left wrist(3))

	// This loop defines the patern of elements in the first row (iGfun=0) of gradient matrix
	for (unsigned int i = 0 ; i < 16 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = 0 ;    // row index
		W_STATIC_vars.jGvar[i] = i ;   // column index
	}
	
	/*
	// This loop defines the patern of elements from i=16 to i=53 which are related to the equation of constraints on joint angles.
	// For joint angle constraints: iGfun=1 to 38 
	// Each row has one non-zero element
	
	int k = 1 ;
	for (unsigned int i = 16 ; i < 54 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = k ;    // row index
		W_STATIC_vars.jGvar[i] = k - 1 ;   // column index
		k = k + 1 ;
	}
	
	// This loop defines the patern of elements from i=54 to i=91 which are related to the equation of constraints on joint velocity.
	// For velocity constraints: iGfun=39 to 76 
	// Each row has one non-zero element
	
	int kk = 39 ;
	for (unsigned int i = 54 ; i < 92 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = kk ;    // row index
		W_STATIC_vars.jGvar[i] = kk - 39 ;   // column index
		kk = kk + 1 ;
	}
	
	
	// This loop defines the patern of elements from i=92 to i=107 which are related to the equation of constraint on center of mass. 
	// For center of mass constraint: iGfun=77 
	
	int kk = 0 ;
	for (unsigned int i = 92; i<108 ; ++i )
	{
		W_STATIC_vars.iGfun[i] = 77 ;
		W_STATIC_vars.jGvar[i] = kk ;
		kk = kk + 1 ;
	}
	
	// This loop defines the patern of elements from i=108 to i=123 which are related to the equation of constraint on y-position of center of mass. 
	// For center of mass constraint: iGfun=78 
	
	int kk = 0 ;
	for (unsigned int i = 108; i<123 ; ++i )
	{
		W_STATIC_vars.iGfun[i] = 78 ;
		W_STATIC_vars.jGvar[i] = kk ;
		kk = kk + 1 ;
	}
	
	
	*/

	//This loop defines the patern of elements related to the equation of constraint on velocity direction.
	// In order to see the effect of this constraint, we removed all other constraint.
	// To solve the general problem, uncomment all above loops and modify the indices of the following loop
	int kk = 0 ;
	for (unsigned int i = 16 ; i<54 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = 1 ;
		W_STATIC_vars.jGvar[i] = kk ;
		kk=kk+1 ;
	}

	snopt_load_vars_full(W_STATIC_vars);
} 

// This function is exactly like the above function (W_STATIC_init) except than initial conditions which come from previous step
void iCubOpt::W_STATIC_update(double x[])
{
	W_STATIC_vars.num_var = 38;
	W_STATIC_vars.num_eq = 2 ; //80; //39;//77;
	W_STATIC_vars.num_deriv = 54 ; //281;//168;// 54 ;//92;  //number of non-zero elements of Gradient Matrix
	W_STATIC_vars.ObjRow  = 0;
	W_STATIC_vars.ObjAdd  = 0; //

	W_STATIC_vars.snopt = new snoptProblemA("W_STATIC","");
	W_STATIC_M = this;

	W_STATIC_vars.snopt->setUserFun(obj);

	snopt_allocate_space(W_STATIC_vars);

	W_STATIC_vars.xlow[0]= -1.000000e-08; W_STATIC_vars.x0[0]= x[0]; W_STATIC_vars.xupp[0]= 1.000000e-08; W_STATIC_vars.xstate[0]= 0;
	W_STATIC_vars.xlow[1]= -1.000000e-08; W_STATIC_vars.x0[1]= x[1]; W_STATIC_vars.xupp[1]= 1.000000e-08; W_STATIC_vars.xstate[1]= 0;
	W_STATIC_vars.xlow[2]= -1.000000e-08; W_STATIC_vars.x0[2]= x[2]; W_STATIC_vars.xupp[2]= 1.000000e-08; W_STATIC_vars.xstate[2]= 0;
	W_STATIC_vars.xlow[3]= -1.000000e-08; W_STATIC_vars.x0[3]= x[3]; W_STATIC_vars.xupp[3]= 1.000000e-08; W_STATIC_vars.xstate[3]= 0;
	W_STATIC_vars.xlow[4]= -1.000000e-08; W_STATIC_vars.x0[4]= x[4]; W_STATIC_vars.xupp[4]= 1.000000e-08; W_STATIC_vars.xstate[4]= 0;
	W_STATIC_vars.xlow[5]= -1.000000e-08; W_STATIC_vars.x0[5]= x[5]; W_STATIC_vars.xupp[5]= 1.000000e-08; W_STATIC_vars.xstate[5]= 0;

	/// torso

	W_STATIC_vars.xlow[6]= -24;	 W_STATIC_vars.x0[6]= x[6]; W_STATIC_vars.xupp[6]= 24; 	  W_STATIC_vars.xstate[6]= 0;
	W_STATIC_vars.xlow[7]= -24;  	 W_STATIC_vars.x0[7]= x[7]; W_STATIC_vars.xupp[7]= 24;    W_STATIC_vars.xstate[7]= 0;
	W_STATIC_vars.xlow[8]= -24;  	 W_STATIC_vars.x0[8]= x[8]; W_STATIC_vars.xupp[8]= 24;    W_STATIC_vars.xstate[8]= 0;

	/// left body

	W_STATIC_vars.xlow[9]=  -12 ;    W_STATIC_vars.x0[9]= x[9];   W_STATIC_vars.xupp[9]= 12; 	W_STATIC_vars.xstate[9]= 0;
	W_STATIC_vars.xlow[10]= -12 ;    W_STATIC_vars.x0[10]= x[10];  W_STATIC_vars.xupp[10]= 12;   W_STATIC_vars.xstate[10]= 0;
	W_STATIC_vars.xlow[11]= -12 ;    W_STATIC_vars.x0[11]= x[11];  W_STATIC_vars.xupp[11]= 12;   W_STATIC_vars.xstate[11]= 0;

	W_STATIC_vars.xlow[12]= -12 ;    W_STATIC_vars.x0[12]= x[12]; W_STATIC_vars.xupp[12]= 12;    W_STATIC_vars.xstate[12]= 0;

	W_STATIC_vars.xlow[13]= -0.45;   W_STATIC_vars.x0[13]= x[13]; W_STATIC_vars.xupp[13]= 0.45;  W_STATIC_vars.xstate[13]= 0;
	W_STATIC_vars.xlow[14]= -0.65;   W_STATIC_vars.x0[14]= x[14]; W_STATIC_vars.xupp[14]= 0.65;  W_STATIC_vars.xstate[14]= 0;
	W_STATIC_vars.xlow[15]= -0.65;   W_STATIC_vars.x0[15]= x[15]; W_STATIC_vars.xupp[15]= 0.65;  W_STATIC_vars.xstate[15]= 0;

	W_STATIC_vars.xlow[16]= -40;     W_STATIC_vars.x0[16]= x[16]; W_STATIC_vars.xupp[16]= 40;    W_STATIC_vars.xstate[16]= 0;
	W_STATIC_vars.xlow[17]= -40; 	 W_STATIC_vars.x0[17]= x[17]; W_STATIC_vars.xupp[17]= 40;    W_STATIC_vars.xstate[17]= 0;
	W_STATIC_vars.xlow[18]= -40;	 W_STATIC_vars.x0[18]= x[18]; W_STATIC_vars.xupp[18]= 40;    W_STATIC_vars.xstate[18]= 0;
	W_STATIC_vars.xlow[19]= -40;     W_STATIC_vars.x0[19]= x[19]; W_STATIC_vars.xupp[19]= 40;     W_STATIC_vars.xstate[19]= 0;
	W_STATIC_vars.xlow[20]= -40;     W_STATIC_vars.x0[20]= x[20]; W_STATIC_vars.xupp[20]= 40;     W_STATIC_vars.xstate[20]= 0;
	W_STATIC_vars.xlow[21]= -40;     W_STATIC_vars.x0[21]= x[21]; W_STATIC_vars.xupp[21]= 40;     W_STATIC_vars.xstate[21]= 0;

	//// right body

	W_STATIC_vars.xlow[22]= -12;      W_STATIC_vars.x0[22]= x[22];   W_STATIC_vars.xupp[22]= 12; 	W_STATIC_vars.xstate[22]= 0;
	W_STATIC_vars.xlow[23]= -12 ;     W_STATIC_vars.x0[23]= x[23];  W_STATIC_vars.xupp[23]= 12;   W_STATIC_vars.xstate[23]= 0;
	W_STATIC_vars.xlow[24]= -12 ;     W_STATIC_vars.x0[24]= x[24];  W_STATIC_vars.xupp[24]= 12;   W_STATIC_vars.xstate[24]= 0;

	W_STATIC_vars.xlow[25]= -12 ;     W_STATIC_vars.x0[25]= x[25]; W_STATIC_vars.xupp[25]= 12;    W_STATIC_vars.xstate[25]= 0;

	W_STATIC_vars.xlow[26]= -0.45;    W_STATIC_vars.x0[26]= x[26]; W_STATIC_vars.xupp[26]= 0.45;  W_STATIC_vars.xstate[26]= 0;
	W_STATIC_vars.xlow[27]= -0.65;    W_STATIC_vars.x0[27]= x[27]; W_STATIC_vars.xupp[27]= 0.65;  W_STATIC_vars.xstate[27]= 0;
	W_STATIC_vars.xlow[28]= -0.65;    W_STATIC_vars.x0[28]= x[28]; W_STATIC_vars.xupp[28]= 0.65;  W_STATIC_vars.xstate[28]= 0;

	W_STATIC_vars.xlow[29]= -40;      W_STATIC_vars.x0[29]= x[29]; W_STATIC_vars.xupp[29]= 40;    W_STATIC_vars.xstate[29]= 0;
	W_STATIC_vars.xlow[30]= -40; 	  W_STATIC_vars.x0[30]= x[30]; W_STATIC_vars.xupp[30]= 40;    W_STATIC_vars.xstate[30]= 0;
	W_STATIC_vars.xlow[31]= -40;	  W_STATIC_vars.x0[31]= x[31]; W_STATIC_vars.xupp[31]= 40;    W_STATIC_vars.xstate[31]= 0;

	W_STATIC_vars.xlow[32]= -40;      W_STATIC_vars.x0[32]= x[32]; W_STATIC_vars.xupp[32]= 40;     W_STATIC_vars.xstate[32]= 0;
	W_STATIC_vars.xlow[33]= -40;      W_STATIC_vars.x0[33]= x[33]; W_STATIC_vars.xupp[33]= 40;     W_STATIC_vars.xstate[33]= 0;
	W_STATIC_vars.xlow[34]= -40;      W_STATIC_vars.x0[34]= x[34]; W_STATIC_vars.xupp[34]= 40;     W_STATIC_vars.xstate[34]= 0;

	/// head

	W_STATIC_vars.xlow[35]= -20;     W_STATIC_vars.x0[35]= x[35]; W_STATIC_vars.xupp[35]= 20;     W_STATIC_vars.xstate[35]= 0;
	W_STATIC_vars.xlow[36]= -20;     W_STATIC_vars.x0[36]= x[36]; W_STATIC_vars.xupp[36]= 20;     W_STATIC_vars.xstate[36]= 0;
	W_STATIC_vars.xlow[37]= -20;     W_STATIC_vars.x0[37]= x[37]; W_STATIC_vars.xupp[37]= 20;     W_STATIC_vars.xstate[37]= 0;

	/////////////////////////////////

	/////////////////////////////////
	// F(0) is the equation of cost function (left end-effector velocity)

	W_STATIC_vars.Flow[0]= -1.000000e+20; W_STATIC_vars.Fupp[0]= 1.000000e+20;  

	// Equation F(1) to F(38) are related to constraints on joints angles. Upper and lower magnitude of joint angles are written based on iCub SDF file. 

	/*	
	W_STATIC_vars.Flow[1]= -1.000000e+20; W_STATIC_vars.Fupp[1]= 1.000000e+20;  // root link
	W_STATIC_vars.Flow[2]= -1.000000e+20; W_STATIC_vars.Fupp[2]= 1.000000e+20;
	W_STATIC_vars.Flow[3]= -1.000000e+20; W_STATIC_vars.Fupp[3]= 1.000000e+20;
	W_STATIC_vars.Flow[4]= -1.000000e+20; W_STATIC_vars.Fupp[4]= 1.000000e+20;
	W_STATIC_vars.Flow[5]= -1.000000e+20; W_STATIC_vars.Fupp[5]= 1.000000e+20;
	W_STATIC_vars.Flow[6]= -1.000000e+20; W_STATIC_vars.Fupp[6]= 1.000000e+20;

	W_STATIC_vars.Flow[7]= -0.383972; W_STATIC_vars.Fupp[7]= 1.46608;            // torso
	W_STATIC_vars.Flow[8]= -0.680678; W_STATIC_vars.Fupp[8]= 0.680678;
	W_STATIC_vars.Flow[9]= -1.02974; W_STATIC_vars.Fupp[9]= 1.02974;

	W_STATIC_vars.Flow[10]= -1.65806; W_STATIC_vars.Fupp[10]= 0.0872665;           // left shoulder
	W_STATIC_vars.Flow[11]= 0.00000; W_STATIC_vars.Fupp[11]= 2.80649;
	W_STATIC_vars.Flow[12]= -0.645772; W_STATIC_vars.Fupp[12]= 1.74533;

	W_STATIC_vars.Flow[13]= 0.0959931; W_STATIC_vars.Fupp[13]= 1.85005;          // left elbow

	W_STATIC_vars.Flow[14]= -0.872665; W_STATIC_vars.Fupp[14]= 0.872665;         // left wrist
	W_STATIC_vars.Flow[15]= -1.13446; W_STATIC_vars.Fupp[15]= 0.174533;
	W_STATIC_vars.Flow[16]= -0.436332; W_STATIC_vars.Fupp[16]= 0.436332;

	W_STATIC_vars.Flow[17]= -0.767945; W_STATIC_vars.Fupp[17]= 2.30383;         // left hip
	W_STATIC_vars.Flow[18]= -0.296706; W_STATIC_vars.Fupp[18]= 2.07694;
	W_STATIC_vars.Flow[19]= -1.37881; W_STATIC_vars.Fupp[19]= 1.37881;

	W_STATIC_vars.Flow[20]= -2.18166; W_STATIC_vars.Fupp[20]= 0.401426;         // left foot
	W_STATIC_vars.Flow[21]= -0.733038; W_STATIC_vars.Fupp[21]= 0.366519;
	W_STATIC_vars.Flow[22]= -0.418879; W_STATIC_vars.Fupp[22]= 0.418879;

	W_STATIC_vars.Flow[23]= -1.65806; W_STATIC_vars.Fupp[23]= 0.0872665;        // right shoulder
	W_STATIC_vars.Flow[24]= 0.00000; W_STATIC_vars.Fupp[24]= 2.80649;
	W_STATIC_vars.Flow[25]= -0.645772; W_STATIC_vars.Fupp[25]= 1.74533;

	W_STATIC_vars.Flow[26]= 0.0959931; W_STATIC_vars.Fupp[26]= 1.85005;          // right elbow

	W_STATIC_vars.Flow[27]= -0.872665; W_STATIC_vars.Fupp[27]= 0.872665;         // right wrist
	W_STATIC_vars.Flow[28]= -1.13446; W_STATIC_vars.Fupp[28]= 0.174533;
	W_STATIC_vars.Flow[29]= -0.436332; W_STATIC_vars.Fupp[29]= 0.436332;

	W_STATIC_vars.Flow[30]= -0.767945; W_STATIC_vars.Fupp[30]= 2.30383;         // right hip
	W_STATIC_vars.Flow[31]= -0.296706; W_STATIC_vars.Fupp[31]= 2.07694;
	W_STATIC_vars.Flow[32]= -1.37881; W_STATIC_vars.Fupp[32]= 1.37881;

	W_STATIC_vars.Flow[33]= -2.18166; W_STATIC_vars.Fupp[33]= 0.401426;         // right foot
	W_STATIC_vars.Flow[34]= -0.733038; W_STATIC_vars.Fupp[34]= 0.366519;
	W_STATIC_vars.Flow[35]= -0.418879; W_STATIC_vars.Fupp[35]= 0.418879;

	W_STATIC_vars.Flow[36]= -0.523599; W_STATIC_vars.Fupp[36]= 0.383972;        // neck
	W_STATIC_vars.Flow[37]= -0.349066; W_STATIC_vars.Fupp[37]= 0.349066;
	W_STATIC_vars.Flow[38]= -0.767945; W_STATIC_vars.Fupp[38]= 0.767945;

	// Equation F(39) to F(76) are related to constraints on joints velocity. 

	for (unsigned int i = 39; i < 45; i++ )
	{
		W_STATIC_vars.Flow[i]= -1.000000e-08;
		W_STATIC_vars.Fupp[i]= 1.000000e-08;
	}

	for (unsigned int i = 45; i < 77; i++ )
	{
		W_STATIC_vars.Flow[i]= -0.785;
		W_STATIC_vars.Fupp[i]= 0.785;
	}
	
	// Equations F(77) and F(78) are related to x and y position of center of mass
	
	W_STATIC_vars.Flow[77]= -5.000000e-02; W_STATIC_vars.Fupp[77]= 15.000000e-02;
	W_STATIC_vars.Flow[78]= -7.000000e-02; W_STATIC_vars.Fupp[78]= 7.000000e-02;

	
	*/

	// In order to see the effect of this constraint, we removed all other constraint.
	// To solve the general problem, uncomment all above bands (from F(1) to F(78)) and modify the index of the following band:
	//  Equation F(79) is related to direction of velocity in the next step

	W_STATIC_vars.Flow[1]= -2.000000e-01; W_STATIC_vars.Fupp[1]= 2.000000e-01;
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Patern of non-zero elements of gradient matrxi should be defined. Since the velocity of left end-effector is function of left upper body,.. 
	// so, the first row of gradient matrix has 16 non-zero elements. (root link(6)+torso(3)+left shoulder(3)+left elbow(1)+left wrist(3))

	// This loop defines the patern of elements in the first row (iGfun=0) of gradient matrix
	for (unsigned int i = 0 ; i < 16 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = 0 ;    // row index
		W_STATIC_vars.jGvar[i] = i ;   // column index
	}
	
	/*
	// This loop defines the patern of elements from i=16 to i=53 which are related to the equation of constraints on joint angles.
	// For joint angle constraints: iGfun=1 to 38 
	// Each row has one non-zero element
	
	int k = 1 ;
	for (unsigned int i = 16 ; i < 54 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = k ;    // row index
		W_STATIC_vars.jGvar[i] = k - 1 ;   // column index
		k = k + 1 ;
	}
	
	// This loop defines the patern of elements from i=54 to i=91 which are related to the equation of constraints on joint velocity.
	// For velocity constraints: iGfun=39 to 76 
	// Each row has one non-zero element
	
	int kk = 39 ;
	for (unsigned int i = 54 ; i < 92 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = kk ;    // row index
		W_STATIC_vars.jGvar[i] = kk - 39 ;   // column index
		kk = kk + 1 ;
	}
	
	
	// This loop defines the patern of elements from i=92 to i=107 which are related to the equation of constraint on x-position of center of mass. 
	// For center of mass constraint: iGfun=77 
	
	int kk = 0 ;
	for (unsigned int i = 92; i<108 ; ++i )
	{
		W_STATIC_vars.iGfun[i] = 77 ;
		W_STATIC_vars.jGvar[i] = kk ;
		kk = kk + 1 ;
	}
	
	// This loop defines the patern of elements from i=108 to i=123 which are related to the equation of constraint on y-position of center of mass. 
	// For center of mass constraint: iGfun=78 
	
	int kk = 0 ;
	for (unsigned int i = 108; i<123 ; ++i )
	{
		W_STATIC_vars.iGfun[i] = 78 ;
		W_STATIC_vars.jGvar[i] = kk ;
		kk = kk + 1 ;
	}
	
	*/

	//This loop defines the patern of elements related to the equation of constraint on velocity direction.
	// In order to see the effect of this constraint, we removed all other constraint.
	// To solve the general problem, uncomment all above loops and modify the indices of the following loop
	int kk = 0 ;
	for (unsigned int i = 16 ; i<54 ; ++i)
	{
		W_STATIC_vars.iGfun[i] = 1 ;
		W_STATIC_vars.jGvar[i] = kk ;
		kk=kk+1 ;
	}

	snopt_load_vars_full(W_STATIC_vars);
}

// This function defines the equation of cost function and constraints.
void iCubOpt::W_STATIC_shot(const double sx[], double sF[],  double sG[] )
{ 

	// optimization variables
	double x1 = sx[0];
	double x2 = sx[1];
	double x3 = sx[2];
	double x4 = sx[3];
	double x5 = sx[4];
	double x6 = sx[5];
	double x7 = sx[6];
	double x8 = sx[7];
	double x9 = sx[8];
	double x10 = sx[9];
	double x11 = sx[10];
	double x12 = sx[11];
	double x13 = sx[12];
	double x14 = sx[13];
	double x15 = sx[14];
	double x16 = sx[15];
	double x17 = sx[16];
	double x18 = sx[17];
	double x19 = sx[18];
	double x20 = sx[19];
	double x21 = sx[20];
	double x22 = sx[21];
	double x23 = sx[22];
	double x24 = sx[23];
	double x25 = sx[24];
	double x26 = sx[25];
	double x27 = sx[26];
	double x28 = sx[27];
	double x29 = sx[28];
	double x30 = sx[29];
	double x31 = sx[30];
	double x32 = sx[31];
	double x33 = sx[32];
	double x34 = sx[33];
	double x35 = sx[34];
	double x36 = sx[35];
	double x37 = sx[36];
	double x38 = sx[37];

	// cost function = [transpose(J * q_dot) ] * (J * q_dot) = tr(q_dot)* tr(J) * J * q_dot
	
	sF[0] = (qd1 + dt * x1) * ((qd1 + dt * x1) * j1_1 + (qd2 + dt * x2) * j2_1 + (qd3 + dt * x3) * j3_1 + (qd4 + dt * x4) * j4_1 + (qd5 + dt * x5) * j5_1 + (qd6 + dt * x6) * j6_1 + (qd7 + dt * x7) * j7_1 + (qd8 + dt * x8) * j8_1 + (qd9 + dt * x9) * j9_1 + (qd10 + dt * x10) * j10_1 + (qd11 + dt * x11) * j11_1 + (qd12+ dt * x12) * j12_1 + (qd13+ dt * x13) * j13_1 + (qd14+ dt * x14) * j14_1 + (qd15+ dt * x15) * j15_1 + (qd16+ dt * x16) * j16_1) + \
		(qd2 + dt * x2) * ((qd1 + dt * x1) * j1_2 + (qd2 + dt * x2) * j2_2 + (qd3 + dt * x3) * j3_2 + (qd4 + dt * x4) * j4_2 + (qd5 + dt * x5) * j5_2 + (qd6 + dt * x6) * j6_2 + (qd7 + dt * x7) * j7_2 + (qd8 + dt * x8) * j8_2 + (qd9 + dt * x9) * j9_2 + (qd10 + dt * x10) * j10_2 + (qd11 + dt * x11) * j11_2 + (qd12+ dt * x12) * j12_2 + (qd13+ dt * x13) * j13_2 + (qd14+ dt * x14) * j14_2 + (qd15+ dt * x15) * j15_2 + (qd16+ dt * x16) * j16_2) + \
		(qd3 + dt * x3) * ((qd1 + dt * x1) * j1_3 + (qd2 + dt * x2) * j2_3 + (qd3 + dt * x3) * j3_3 + (qd4 + dt * x4) * j4_3 + (qd5 + dt * x5) * j5_3 + (qd6 + dt * x6) * j6_3 + (qd7 + dt * x7) * j7_3 + (qd8 + dt * x8) * j8_3 + (qd9 + dt * x9) * j9_3 + (qd10 + dt * x10) * j10_3 + (qd11 + dt * x11) * j11_3 + (qd12+ dt * x12) * j12_3 + (qd13+ dt * x13) * j13_3 + (qd14+ dt * x14) * j14_3 + (qd15+ dt * x15) * j15_3 + (qd16+ dt * x16) * j16_3) + \
		(qd4 + dt * x4) * ((qd1 + dt * x1) * j1_4 + (qd2 + dt * x2) * j2_4 + (qd3 + dt * x3) * j3_4 + (qd4 + dt * x4) * j4_4 + (qd5 + dt * x5) * j5_4 + (qd6 + dt * x6) * j6_4 + (qd7 + dt * x7) * j7_4 + (qd8 + dt * x8) * j8_4 + (qd9 + dt * x9) * j9_4 + (qd10 + dt * x10) * j10_4 + (qd11 + dt * x11) * j11_4 + (qd12+ dt * x12) * j12_4 + (qd13+ dt * x13) * j13_4 + (qd14+ dt * x14) * j14_4 + (qd15+ dt * x15) * j15_4 + (qd16+ dt * x16) * j16_4) + \
		(qd5 + dt * x5) * ((qd1 + dt * x1) * j1_5 + (qd2 + dt * x2) * j2_5 + (qd3 + dt * x3) * j3_5 + (qd4 + dt * x4) * j4_5 + (qd5 + dt * x5) * j5_5 + (qd6 + dt * x6) * j6_5 + (qd7 + dt * x7) * j7_5 + (qd8 + dt * x8) * j8_5 + (qd9 + dt * x9) * j9_5 + (qd10 + dt * x10) * j10_5 + (qd11 + dt * x11) * j11_5 + (qd12+ dt * x12) * j12_5 + (qd13+ dt * x13) * j13_5 + (qd14+ dt * x14) * j14_5 + (qd15+ dt * x15) * j15_5 + (qd16+ dt * x16) * j16_5) + \
		(qd6 + dt * x6) * ((qd1 + dt * x1) * j1_6 + (qd2 + dt * x2) * j2_6 + (qd3 + dt * x3) * j3_6 + (qd4 + dt * x4) * j4_6 + (qd5 + dt * x5) * j5_6 + (qd6 + dt * x6) * j6_6 + (qd7 + dt * x7) * j7_6 + (qd8 + dt * x8) * j8_6 + (qd9 + dt * x9) * j9_6 + (qd10 + dt * x10) * j10_6 + (qd11 + dt * x11) * j11_6 + (qd12+ dt * x12) * j12_6 + (qd13+ dt * x13) * j13_6 + (qd14+ dt * x14) * j14_6 + (qd15+ dt * x15) * j15_6 + (qd16+ dt * x16) * j16_1) + \
		(qd7 + dt * x7) * ((qd1 + dt * x1) * j1_7 + (qd2 + dt * x2) * j2_7 + (qd3 + dt * x3) * j3_7 + (qd4 + dt * x4) * j4_7 + (qd5 + dt * x5) * j5_7 + (qd6 + dt * x6) * j6_7 + (qd7 + dt * x7) * j7_7 + (qd8 + dt * x8) * j8_7 + (qd9 + dt * x9) * j9_7 + (qd10 + dt * x10) * j10_7 + (qd11 + dt * x11) * j11_7 + (qd12+ dt * x12) * j12_7 + (qd13+ dt * x13) * j13_7 + (qd14+ dt * x14) * j14_7 + (qd15+ dt * x15) * j15_7 + (qd16+ dt * x16) * j16_1) + \
		(qd8 + dt * x8) * ((qd1 + dt * x1) * j1_8 + (qd2 + dt * x2) * j2_8 + (qd3 + dt * x3) * j3_8 + (qd4 + dt * x4) * j4_8 + (qd5 + dt * x5) * j5_8 + (qd6 + dt * x6) * j6_8 + (qd7 + dt * x7) * j7_8 + (qd8 + dt * x8) * j8_8 + (qd9 + dt * x9) * j9_8 + (qd10 + dt * x10) * j10_8 + (qd11 + dt * x11) * j11_8 + (qd12+ dt * x12) * j12_8 + (qd13+ dt * x13) * j13_8 + (qd14+ dt * x14) * j14_8 + (qd15+ dt * x15) * j15_8 + (qd16+ dt * x16) * j16_1) + \
		(qd9 + dt * x9) * ((qd1 + dt * x1) * j1_9 + (qd2 + dt * x2) * j2_9 + (qd3 + dt * x3) * j3_9 + (qd4 + dt * x4) * j4_9 + (qd5 + dt * x5) * j5_9 + (qd6 + dt * x6) * j6_9 + (qd7 + dt * x7) * j7_9 + (qd8 + dt * x8) * j8_9 + (qd9 + dt * x9) * j9_9 + (qd10 + dt * x10) * j10_9 + (qd11 + dt * x11) * j11_9 + (qd12+ dt * x12) * j12_9 + (qd13+ dt * x13) * j13_9 + (qd14+ dt * x14) * j14_9 + (qd15+ dt * x15) * j15_9 + (qd16+ dt * x16) * j16_1) + \
		(qd10 + dt * x10) * ((qd1 + dt * x1) * j1_10 + (qd2 + dt * x2) * j2_10 + (qd3 + dt * x3) * j3_10 + (qd4 + dt * x4) * j4_10 + (qd5 + dt * x5) * j5_10 + (qd6 + dt * x6) * j6_10 + (qd7 + dt * x7) * j7_10 + (qd8 + dt * x8) * j8_10 + (qd9 + dt * x9) * j9_10 + (qd10 + dt * x10) * j10_10 + (qd11 + dt * x11) * j11_10 + (qd12+ dt * x12) * j12_10 + (qd13+ dt * x13) * j13_10 + (qd14+ dt * x14) * j14_10 + (qd15+ dt * x15) * j15_10 + (qd16+ dt * x16) * j16_10) + \
		(qd11 + dt * x11) * ((qd1 + dt * x1) * j1_11 + (qd2 + dt * x2) * j2_11 + (qd3 + dt * x3) * j3_11 + (qd4 + dt * x4) * j4_11 + (qd5 + dt * x5) * j5_11 + (qd6 + dt * x6) * j6_11 + (qd7 + dt * x7) * j7_11 + (qd8 + dt * x8) * j8_11 + (qd9 + dt * x9) * j9_11 + (qd10 + dt * x10) * j10_11 + (qd11 + dt * x11) * j11_11 + (qd12+ dt * x12) * j12_11 + (qd13+ dt * x13) * j13_11 + (qd14+ dt * x14) * j14_11 + (qd15+ dt * x15) * j15_11 + (qd16+ dt * x16) * j16_11) + \
		(qd12 + dt * x12) * ((qd1 + dt * x1) * j1_12 + (qd2 + dt * x2) * j2_12 + (qd3 + dt * x3) * j3_12 + (qd4 + dt * x4) * j4_12 + (qd5 + dt * x5) * j5_12 + (qd6 + dt * x6) * j6_12 + (qd7 + dt * x7) * j7_12 + (qd8 + dt * x8) * j8_12 + (qd9 + dt * x9) * j9_12 + (qd10 + dt * x10) * j10_12 + (qd11 + dt * x11) * j11_12 + (qd12+ dt * x12) * j12_12 + (qd13+ dt * x13) * j13_12 + (qd14+ dt * x14) * j14_12 + (qd15+ dt * x15) * j15_12 + (qd16+ dt * x16) * j16_12) + \
		(qd13 + dt * x13) * ((qd1 + dt * x1) * j1_13 + (qd2 + dt * x2) * j2_13 + (qd3 + dt * x3) * j3_13 + (qd4 + dt * x4) * j4_13 + (qd5 + dt * x5) * j5_13 + (qd6 + dt * x6) * j6_13 + (qd7 + dt * x7) * j7_13 + (qd8 + dt * x8) * j8_13 + (qd9 + dt * x9) * j9_13 + (qd10 + dt * x10) * j10_13 + (qd11 + dt * x11) * j11_13 + (qd12+ dt * x12) * j12_13 + (qd13+ dt * x13) * j13_13 + (qd14+ dt * x14) * j14_13 + (qd15+ dt * x15) * j15_13 + (qd16+ dt * x16) * j16_13) + \
		(qd14 + dt * x14) * ((qd1 + dt * x1) * j1_14 + (qd2 + dt * x2) * j2_14 + (qd3 + dt * x3) * j3_14 + (qd4 + dt * x4) * j4_14 + (qd5 + dt * x5) * j5_14 + (qd6 + dt * x6) * j6_11 + (qd7 + dt * x7) * j7_14 + (qd8 + dt * x8) * j8_14 + (qd9 + dt * x9) * j9_14 + (qd10 + dt * x10) * j10_14 + (qd11 + dt * x11) * j11_14 + (qd12+ dt * x12) * j12_14 + (qd13+ dt * x13) * j13_14 + (qd14+ dt * x14) * j14_14 + (qd15+ dt * x15) * j15_14 + (qd16+ dt * x16) * j16_14) + \
		(qd15 + dt * x15) * ((qd1 + dt * x1) * j1_15 + (qd2 + dt * x2) * j2_15 + (qd3 + dt * x3) * j3_15 + (qd4 + dt * x4) * j4_15 + (qd5 + dt * x5) * j5_15 + (qd6 + dt * x6) * j6_12 + (qd7 + dt * x7) * j7_15 + (qd8 + dt * x8) * j8_15 + (qd9 + dt * x9) * j9_15 + (qd10 + dt * x10) * j10_15 + (qd11 + dt * x11) * j11_15 + (qd12+ dt * x12) * j12_15 + (qd13+ dt * x13) * j13_15 + (qd14+ dt * x14) * j14_15 + (qd15+ dt * x15) * j15_15 + (qd16+ dt * x16) * j16_15) + \
		(qd16 + dt * x16) * ((qd1 + dt * x1) * j1_16 + (qd2 + dt * x2) * j2_16 + (qd3 + dt * x3) * j3_16 + (qd4 + dt * x4) * j4_16 + (qd5 + dt * x5) * j5_16 + (qd6 + dt * x6) * j6_13 + (qd7 + dt * x7) * j7_16 + (qd8 + dt * x8) * j8_16 + (qd9 + dt * x9) * j9_16 + (qd10 + dt * x10) * j10_16 + (qd11 + dt * x11) * j11_16 + (qd12+ dt * x12) * j12_16 + (qd13+ dt * x13) * j13_16 + (qd14+ dt * x14) * j14_16 + (qd15+ dt * x15) * j15_16 + (qd16+ dt * x16) * j16_16) ;

	sF[0] = -sF[0];

	pos[0] = q1 ;
	pos[1] = q2 ;
	pos[2] = q3 ;
	pos[3] = q4 ;
	pos[4] = q5 ;
	pos[5] = q6 ;
	pos[6] = q7 ;
	pos[7] = q8 ;
	pos[8] = q9 ;
	pos[9] = q10 ;
	pos[10] = q11 ;
	pos[11] = q12 ;
	pos[12] = q13 ;
	pos[13] = q14 ;
	pos[14] = q15 ;
	pos[15] = q16 ;
	pos[16] = q17 ;
	pos[17] = q18 ;
	pos[18] = q19 ;
	pos[19] = q20 ;
	pos[20] = q21 ;
	pos[21] = q22 ;
	pos[22] = q23 ;
	pos[23] = q24 ;
	pos[24] = q25 ;
	pos[25] = q26 ;
	pos[26] = q27 ;
	pos[27] = q28 ;
	pos[28] = q29 ;
	pos[29] = q30 ;
	pos[30] = q31 ;
	pos[31] = q32 ;
	pos[32] = q33 ;
	pos[33] = q34 ;
	pos[34] = q35 ;
	pos[35] = q36 ;
	pos[36] = q37 ;
	pos[37] = q38 ;

	vel[0] = qd1 ;
	vel[1] = qd2 ;
	vel[2] = qd3 ;
	vel[3] = qd4 ;
	vel[4] = qd5 ;
	vel[5] = qd6 ;
	vel[6] = qd7 ;
	vel[7] = qd8 ;
	vel[8] = qd9 ;
	vel[9] = qd10 ;
	vel[10] = qd11 ;
	vel[11] = qd12 ;
	vel[12] = qd13 ;
	vel[13] = qd14 ;
	vel[14] = qd15 ;
	vel[15] = qd16 ;
	vel[16] = qd17 ;
	vel[17] = qd18 ;
	vel[18] = qd19 ;
	vel[19] = qd20 ;
	vel[20] = qd21 ;
	vel[21] = qd22 ;
	vel[22] = qd23 ;
	vel[23] = qd24 ;
	vel[24] = qd25 ;
	vel[25] = qd26 ;
	vel[26] = qd27 ;
	vel[27] = qd28 ;
	vel[28] = qd29 ;
	vel[29] = qd30 ;
	vel[30] = qd31 ;
	vel[31] = qd32 ;
	vel[32] = qd33 ;
	vel[33] = qd34 ;
	vel[34] = qd35 ;
	vel[35] = qd36 ;
	vel[36] = qd37 ;
	vel[37] = qd38 ;


	iCubModel.set_state(0, pos, vel);
	//iCubModel.set_acc(joint_acc) ;
	
	// 11 is the index of palm (end effector)
	end_pos = iCubModel.get_pos(11, Cvector3(0,0,0));
	// std::cout << "Initial Position = " << end_pos.transpose() << std::endl;
	Cvector diff = r_D - end_pos ;
	//   std::cout << "Initial diff = " << diff.transpose() << std::endl;

	double diff_norm = sqrt( diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] );
	diff = diff / diff_norm ;

	// Constraints on joint angles
	sF[1] =  q1 + dt * ( qd1 + dt * x1 );
	sF[2] =  q2 + dt * ( qd2 + dt * x2 );
	sF[3] =  q3 + dt * ( qd3 + dt * x3 );
	sF[4] =  q4 + dt * ( qd4 + dt * x4 );
	sF[5] =  q5 + dt * ( qd5 + dt * x5 );
	sF[6] =  q6 + dt * ( qd6 + dt * x6 );
	sF[7] =  q7 + dt * ( qd7 + dt * x7 );
	sF[8] =  q8 + dt * ( qd8 + dt * x8 );
	sF[9] =  q9 + dt * ( qd9 + dt * x9 );
	sF[10] = q10 + dt * ( qd10 + dt * x10 );
	sF[11] = q11 + dt * ( qd11 + dt * x11 );
	sF[12] = q12 + dt * ( qd12 + dt * x12 );
	sF[13] = q13 + dt * ( qd13 + dt * x13 );
	sF[14] = q14 + dt * ( qd14 + dt * x14 );
	sF[15] = q15 + dt * ( qd15 + dt * x15 );
	sF[16] = q16 + dt * ( qd16 + dt * x16 );
	sF[17] = q17 + dt * ( qd17 + dt * x17 );
	sF[18] = q18 + dt * ( qd18 + dt * x18 );
	sF[19] = q19 + dt * ( qd19 + dt * x19 );
	sF[20] = q20 + dt * ( qd20 + dt * x20 );
	sF[21] = q21 + dt * ( qd21 + dt * x21 );
	sF[22] = q22 + dt * ( qd22 + dt * x22 );
	sF[23] = q23 + dt * ( qd23 + dt * x23 );
	sF[24] = q24 + dt * ( qd24 + dt * x24 );
	sF[25] = q25 + dt * ( qd25 + dt * x25 );
	sF[26] = q26 + dt * ( qd26 + dt * x26 );
	sF[27] = q27 + dt * ( qd27 + dt * x27 );
	sF[28] = q28 + dt * ( qd28 + dt * x28 );
	sF[29] = q29 + dt * ( qd29 + dt * x29 );
	sF[30] = q30 + dt * ( qd30 + dt * x30 );
	sF[31] = q31 + dt * ( qd31 + dt * x31 );
	sF[32] = q32 + dt * ( qd32 + dt * x32 );
	sF[33] = q33 + dt * ( qd33 + dt * x33 );
	sF[34] = q34 + dt * ( qd34 + dt * x34 );
	sF[35] = q35 + dt * ( qd35 + dt * x35 );
	sF[36] = q36 + dt * ( qd36 + dt * x36 );
	sF[37] = q37 + dt * ( qd37 + dt * x37 );
	sF[38] = q38 + dt * ( qd38 + dt * x38 );

	// constraint on joint velocity
	sF[39] =  qd1 + dt * x1 ;
	sF[40] =  qd2 + dt * x2 ;
	sF[41] =  qd3 + dt * x3 ;
	sF[42] =  qd4 + dt * x4 ;
	sF[43] =  qd5 + dt * x5 ;
	sF[44] =  qd6 + dt * x6 ;
	sF[45] =  qd7 + dt * x7 ;
	sF[46] =  qd8 + dt * x8 ;
	sF[47] =  qd9 + dt * x9 ;
	sF[48] =  qd10 + dt * x10 ;
	sF[49] =  qd11 + dt * x11 ;
	sF[50] =  qd12 + dt * x12 ;
	sF[51] =  qd13 + dt * x13 ;
	sF[52] =  qd14 + dt * x14 ;
	sF[53] =  qd15 + dt * x15 ;
	sF[54] =  qd16 + dt * x16 ;
	sF[55] =  qd17 + dt * x17 ;
	sF[56] =  qd18 + dt * x18 ;
	sF[57] =  qd19 + dt * x19 ;
	sF[58] =  qd20 + dt * x20 ;
	sF[59] =  qd21 + dt * x21 ;
	sF[60] =  qd22 + dt * x22 ;
	sF[61] =  qd23 + dt * x23 ;
	sF[62] =  qd24 + dt * x24 ;
	sF[63] =  qd25 + dt * x25 ;
	sF[64] =  qd26 + dt * x26 ;
	sF[65] =  qd27 + dt * x27 ;
	sF[66] =  qd28 + dt * x28 ;
	sF[67] =  qd29 + dt * x29 ;
	sF[68] =  qd30 + dt * x30 ;
	sF[69] =  qd31 + dt * x31 ;
	sF[70] =  qd32 + dt * x32 ;
	sF[71] =  qd33 + dt * x33 ;
	sF[72] =  qd34 + dt * x34 ;
	sF[73] =  qd35 + dt * x35 ;
	sF[74] =  qd36 + dt * x36 ;
	sF[75] =  qd37 + dt * x37 ;
	sF[76] =  qd38 + dt * x38 ;

	for (unsigned int i=0;i<38;++i)
	{
		pos_new(i)=sF[i+1] ;
	}

	for (unsigned int i=0;i<38;++i)
	{
		vel_new(i)=sF[i+39] ;
	}

	end_vel(0)=0;
	end_vel(1)=0;
	end_vel(2)=0;

	iCubModel.set_state(0, pos_new, vel_new);
	end_vel = iCubModel.get_vel(11, Cvector3(0,0,0));
	
	/*
	Cvector3 com =  iCubModel.get_cm()  ;

	com_x = com[0] ;
	com_y = com[1] ;

	sF[77] = com_x ;
        sF[78] = com_y ;
	 */
	
	//  std::cout << "New q = "<< pos_new.transpose() << std::endl;
	//  std::cout << "vel_new = "<< vel_new.transpose() << std::endl;
	// std::cout << "Next Velocity = "<< end_vel.transpose() << std::endl;
	//  std::cout <<"Next Position = " <<iCubModel.get_pos(11, Cvector3(0,0,0)).transpose() << std::endl;


	double vel_norm = sqrt( end_vel[0]*end_vel[0] + end_vel[1]*end_vel[1] + end_vel[2]*end_vel[2] );

	//   std::cout << "q_dot_elbow = " << vel[12] <<std::endl ;

	// std::cout << "Final Position  = " << iCubModel.get_pos(11, Cvector3(0,0,0)).transpose() <<std::endl ;

	end_vel = end_vel / vel_norm ;

	dir_cons = diff - end_vel ;
	double dir_norm = sqrt( dir_cons[0]*dir_cons[0] + dir_cons[1]*dir_cons[1] + dir_cons[2]*dir_cons[2] );
	// std::cout << "Direction_norm = " << dir_norm <<std::endl ;


	sF[1]=  dir_norm ;




	/* for (unsigned int j = 16; j < 54; j++ )
    {
    	sG[j]= dt * dt;
    }

    for (unsigned int j = 54; j < 92; j++ )
    {
    	sG[j]= dt;
    }*/
} 
