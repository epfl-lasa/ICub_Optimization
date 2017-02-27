#include "Model.hpp"
using namespace std;

int main()
{

    Model model;

    model.init();


	Cvector ref_pos = Cvector::Zero(AIR_N_Q);
	// height of root link is set here to be consistent with root link in gazebo
	ref_pos[2] = 0.6 ; // height of root link

	// set initial joint angles
	/*ref_pos[9] = -0.52 ;    //left shoulder pitch,
	ref_pos[10] = 0.52 ;    //left shoulder roll,
	ref_pos[12] = 0.785 ;    // l_elbow
	ref_pos[15] = 0.436332 ;    // l_wrist_yaw

	ref_pos[22] = -0.52 ;
	ref_pos[23] = 0.52 ;
	ref_pos[25] = 0.785 ;
	ref_pos[28] = 0.436332 ;*/


	ref_pos[12] = 0.0959931 ;    // l_elbow
	ref_pos[25] = 0.0959931 ;    // r_elbow
	/*ref_pos[15] = 0.436332 ;    // l_wrist_yaw
	ref_pos[28] = 0.436332 ;    // l_wrist_yaw*/


	Cvector ref_vel = Cvector::Zero(AIR_N_U);

    std::ofstream jacobian_mat ;
    jacobian_mat.open( "jacobian_mat.txt", std::ofstream::out | std::ofstream::app );


             model.set_state(0, ref_pos, ref_vel);
         //  model.check_consistency(Cvector::Random(AIR_N_Q),Cvector::Random(AIR_N_U),Cvector::Random(AIR_N_U));


   Cmatrix jac_tr = model.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION);
   Cmatrix jac_rot = model.get_jacob(11, Cvector3(0,0,0), CT_ROTATION);
/*

/// print desired elements
   for(unsigned int i = 0; i < 16; ++i)
   {
	   jacobian_mat << jac_tr(0,i) << "  "   ;
   }

   jacobian_mat << endl << endl ;
   for(unsigned int i = 0; i < 16; ++i)
      {
   	   jacobian_mat << jac_tr(1,i) << "  "   ;
      }
   jacobian_mat << endl <<endl ;

   for(unsigned int i = 0; i < 16; ++i)
      {
   	   jacobian_mat << jac_tr(2,i) << "  "   ;
      }
   jacobian_mat << endl <<endl <<endl <<endl <<endl ;
/////////////////
   for(unsigned int i = 0; i < 16; ++i)
      {
   	   jacobian_mat << jac_rot(0,i) << "  "   ;
      }

      jacobian_mat << endl << endl ;
      for(unsigned int i = 0; i < 16; ++i)
         {
      	   jacobian_mat << jac_rot(1,i) << "  "   ;
         }
      jacobian_mat << endl <<endl ;

      for(unsigned int i = 0; i < 16; ++i)
         {
      	   jacobian_mat << jac_rot(2,i) << "  "   ;
         }
/// end of print

*/

   //cout << model.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION) << endl << endl;


  // cout << model.get_jacob(11, Cvector3(0,0,0), CT_ROTATION) << endl << endl;

 //  cout << model.get_massmat() << endl;

   cout << model.get_massmat() << endl;

   cout << model.get_cm().transpose() << endl;

   for (unsigned int j = 0;j < 39; ++j )
   {
	   std::cout << j << "---Link Position = " << model.get_pos(j, Cvector3(0,0,0)).transpose() << endl << endl;

   }

   std::cout << model.get_pos(-1, Cvector3(0,0,0)) << std::endl;
    return 0;
}
