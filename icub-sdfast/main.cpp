#include "Model.hpp"
using namespace std;

int main()
{

    Model model;

    model.init();


	Cvector ref_pos = Cvector::Zero(AIR_N_U);
	// height of root link is set here to be consistent with root link in gazebo
	ref_pos[2] = 0.6 ; // height of root link

	ref_pos[12] = 0.959931 ;    // l_elbow
	ref_pos[25] = 0.959931 ;    // r_elbow

	// set initial joint angles
	/*ref_pos[9] = -0.52 ;    //left shoulder pitch,
	ref_pos[10] = 0.52 ;    //left shoulder roll,
	ref_pos[12] = 0.785 ;    // l_elbow
	ref_pos[15] = 0.436332 ;    // l_wrist_yaw

	ref_pos[22] = -0.52 ;
	ref_pos[23] = 0.52 ;
	ref_pos[25] = 0.785 ;
	ref_pos[28] = 0.436332 ;*/

	//ref_pos[8] = 1.85005 ;    // l_elbow
	//ref_pos[24] = 1.85005 ;    // r_elbow


	/*ref_pos[15] = 0.436332 ;    // l_wrist_yaw
	ref_pos[28] = 0.436332 ;    // l_wrist_yaw*/

	Cvector ref_vel = Cvector::Zero(AIR_N_U);
	   model.set_state(0, ref_pos, ref_vel);

    std::ofstream jacobian_mat ;
    jacobian_mat.open( "jacobian_mat.txt", std::ofstream::out | std::ofstream::app );




   Cmatrix jac_tr = model.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION);
   Cmatrix jac_rot = model.get_jacob(11, Cvector3(0,0,0), CT_ROTATION);

   Cvector3 end_vel = jac_tr * ref_vel ;

   std::cout << "Velocity end effector = "<< end_vel.transpose() << std::endl;
   std::cout << "Velocity end effector 2  = "<< model.get_vel(11,Cvector3(0,0,0) ).transpose() << std::endl;




   cout << "Mass Matrix = "<<model.get_massmat() << endl;


   cout << "Center of Gravity = "<<model.get_cm().transpose() << endl;

   for (unsigned int j = 0;j < 39; ++j )
   {
	   std::cout << j << "---Link Position = " << model.get_pos(j, Cvector3(0,0,0)).transpose() << endl << endl;

   }



    return 0;
}
