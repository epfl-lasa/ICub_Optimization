#include "iCubOpt.hpp"
 
void iCubOpt::init()
{ 

	// Initialize iCub Model from SD-Fast
	iCubModel.init();


	Cvector pos_cur = Cvector::Zero(AIR_N_Q);
	pos_cur[2] = 0.6 ; // height of root link

/*
	// set initial joint angles
	pos_cur[9] = -0.52 ;    //left shoulder pitch,
	pos_cur[10] = 0.52 ;    //left shoulder roll,
	pos_cur[12] = 0.785 ;    // l_elbow
	pos_cur[15] = 0.436332 ;    // l_wrist_yaw

	pos_cur[22] = -0.52 ;
	pos_cur[23] = 0.52 ;
	pos_cur[25] = 0.785 ;
	pos_cur[28] = 0.436332 ;
*/

	pos_cur[12] = 0.0959931 ;    // l_elbow
	pos_cur[25] = 0.0959931 ;    // l_elbow

	Cvector vel_cur = Cvector::Zero(AIR_N_U);
	iCubModel.set_state(0, pos_cur, vel_cur);

	//iCubModel.check_consistency(Cvector::Random(AIR_N_Q),Cvector::Random(AIR_N_U),Cvector::Random(AIR_N_U));
	//std::cout << iCubModel.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION) << std::endl << std::endl;

	//std::cout << iCubModel.get_jacob(11, Cvector3(0,0,0), CT_ROTATION) << std::endl << std::endl;

	//std::cout << iCubModel.get_massmat() << std::endl;
	//std::cout << "HAND POSITION =  " << iCubModel.get_pos(11, Cvector3(0,0,0) ) << std::endl ;

	std::ofstream opt_data ;
	opt_data.open( "opt_data.txt", std::ofstream::out | std::ofstream::app );

	opt_data << "Initial COM Position = " << iCubModel.get_cm().transpose() << std::endl ;
	opt_data <<  "Initial end eff. position = "<< iCubModel.get_pos(11, Cvector3(0,0,0)).transpose() << std::endl;
	opt_data <<  "Initial end eff. velocity = "<< iCubModel.get_vel(11, Cvector3(0,0,0)).transpose() << std::endl;

	Cvector3 velocity_n  = iCubModel.get_vel(11, Cvector3(0,0,0));
    velocity_norm = sqrt( velocity_n[0]*velocity_n[0] + velocity_n[1]*velocity_n[1] + velocity_n[2]*velocity_n[2] );


	Cmatrix jac_tr  = iCubModel.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION);
	Cmatrix jac_rot = iCubModel.get_jacob(11, Cvector3(0,0,0), CT_ROTATION);

	//std::cout << jac_tr << std::endl;

	Cvector3 vel_new = jac_tr *  vel_cur ;

	//opt_data <<  "Initial end eff. velocity_new = "<< vel_new.transpose() << std::endl;

	//std::cout << iCubModel.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION) << std::endl;

/*
	Cvector3 end_pos = iCubModel.get_pos(11, Cvector3(0,0,0));
	Cvector3 r_D = Cvector3(0,0,0);
	r_D[0] = rd_x ;
	r_D[1] = rd_y ;
	r_D[2] = rd_z ;

	Cvector diff = r_D - end_pos ;
	double diff_norm = sqrt( diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] );
	diff = diff / diff_norm ;

	Cvector3 end_vel = iCubModel.get_vel(11, Cvector3(0,0,0));
	//double vel_norm = sqrt( end_vel[0]*end_vel[0] + end_vel[1]*end_vel[1] + end_vel[2]*end_vel[2] );
	end_vel = end_vel ; // / vel_norm ;

	Cvector dir_cons = diff - end_vel ;
	dir_norm = sqrt( dir_cons[0]*dir_cons[0] + dir_cons[1]*dir_cons[1] + dir_cons[2]*dir_cons[2] );*/

	Cmatrix jj = jac_tr.transpose();

    jj = jj * jac_tr ;
    //  std::cout << jj(15,16) << std::endl;
    //std::cout << jac_tr.size() << std::endl ;



    q1 = pos_cur[0];
    q2 = pos_cur[1];
    q3 = pos_cur[2];
    q4 = pos_cur[3];
    q5 = pos_cur[4];
    q6 = pos_cur[5];
    q7 = pos_cur[6];
    q8 = pos_cur[7];
    q9 = pos_cur[8];
    q10 = pos_cur[9];
    q11 = pos_cur[10];
    q12 = pos_cur[11];
    q13 = pos_cur[12];
    q14 = pos_cur[13];
    q15 = pos_cur[14];
    q16 = pos_cur[15];
    q17 = pos_cur[16];
    q18 = pos_cur[17];
    q19 = pos_cur[18];
    q20 = pos_cur[19];
    q21 = pos_cur[20];
    q22 = pos_cur[21];
    q23 = pos_cur[22];
    q24 = pos_cur[23];
    q25 = pos_cur[24];
    q26 = pos_cur[25];
    q27 = pos_cur[26];
    q28 = pos_cur[27];
    q29 = pos_cur[28];
    q30 = pos_cur[29];
    q31 = pos_cur[30];
    q32 = pos_cur[31];
    q33 = pos_cur[32];
    q34 = pos_cur[33];
    q35 = pos_cur[34];
    q36 = pos_cur[35];
    q37 = pos_cur[36];
    q38 = pos_cur[37];


    qd1 = vel_cur[0];
    qd2 = vel_cur[1];
    qd3 = vel_cur[2];
    qd4 = vel_cur[3];
    qd5 = vel_cur[4];
    qd6 = vel_cur[5];
    qd7 = vel_cur[6];
    qd8 = vel_cur[7];
    qd9 = vel_cur[8];
    qd10 = vel_cur[9];
    qd11 = vel_cur[10];
    qd12 = vel_cur[11];
    qd13 = vel_cur[12];
    qd14 = vel_cur[13];
    qd15 = vel_cur[14];
    qd16 = vel_cur[15];
    qd17 = vel_cur[16];
    qd18 = vel_cur[17];
    qd19 = vel_cur[18];
    qd20 = vel_cur[19];
    qd21 = vel_cur[20];
    qd22 = vel_cur[21];
    qd23 = vel_cur[22];
    qd24 = vel_cur[23];
    qd25 = vel_cur[24];
    qd26 = vel_cur[25];
    qd27 = vel_cur[26];
    qd28 = vel_cur[27];
    qd29 = vel_cur[28];
    qd30 = vel_cur[29];
    qd31 = vel_cur[30];
    qd32 = vel_cur[31];
    qd33 = vel_cur[32];
    qd34 = vel_cur[33];
    qd35 = vel_cur[34];
    qd36 = vel_cur[35];
    qd37 = vel_cur[36];
    qd38 = vel_cur[37];


}

void iCubOpt::update(double x[])
{

	// Initialize iCub Model from SD-Fast
	//iCubModel.init();

	qd1  = qd1 + dt * x[0];
	q1	 = q1 + dt * qd1 ;

	qd2  = qd2 + dt * x[1];
	q2	 = q2 + dt * qd2 ;

	qd3  = qd3 + dt * x[2];
	q3	 = q3 + dt * qd3 ;

	qd4  = qd4 + dt * x[3];
	q4	 = q4 + dt * qd4 ;

	qd5  = qd5 + dt * x[4];
	q5	 = q5 + dt * qd5 ;

	qd6  = qd6 + dt * x[5];
	q6	 = q6 + dt * qd6 ;

	qd7  = qd7 + dt * x[6];
	q7	 = q7 + dt * qd7 ;

	qd8  = qd8 + dt * x[7];
	q8	 = q8 + dt * qd8 ;

	qd7  = qd7 + dt * x[8];
	q7	 = q7 + dt * qd9 ;

	qd10  = qd10 + dt * x[9];
	q10	 = q10 + dt * qd10 ;

	qd11  = qd11 + dt * x[10];
	q11	 = q11 + dt * qd11 ;

	qd12  = qd12 + dt * x[11];
	q12	 = q12 + dt * qd12 ;

	qd13  = qd13 + dt * x[12];
	q13	 = q13 + dt * qd13 ;

	qd14  = qd14 + dt * x[13];
	q14	 = q14 + dt * qd14 ;

	qd15  = qd15 + dt * x[14];
	q15	 = q15 + dt * qd15 ;

	qd16  = qd16 + dt * x[15];
	q16	 = q16 + dt * qd16 ;

	qd17  = qd17 + dt * x[16];
	q17	 = q17 + dt * qd17 ;

	qd18  = qd18 + dt * x[17];
	q18	 = q18 + dt * qd18 ;

	qd19  = qd19 + dt * x[18];
	q19	 = q19 + dt * qd19 ;

	qd20  = qd20 + dt * x[19];
	q20	 = q20 + dt * qd20 ;

	qd21  = qd21 + dt * x[20];
	q21	 = q21 + dt * qd21 ;

	qd22  = qd22 + dt * x[21];
	q22	 = q22 + dt * qd20 ;

	qd23  = qd23 + dt * x[22];
	q23	 = q23 + dt * qd23 ;

	qd24  = qd24 + dt * x[23];
	q24	 = q24 + dt * qd24 ;

	qd25  = qd25 + dt * x[24];
	q25	 = q25 + dt * qd25 ;

	qd26  = qd26 + dt * x[25];
	q26	 = q26 + dt * qd26 ;

	qd27  = qd27 + dt * x[26];
	q27	 = q27 + dt * qd27 ;

	qd28  = qd28 + dt * x[27];
	q28	 = q28 + dt * qd28 ;

	qd29  = qd29 + dt * x[28];
	q29	 = q29 + dt * qd29 ;

	qd30  = qd30 + dt * x[29];
	q30	 = q30 + dt * qd30 ;

	qd31  = qd31 + dt * x[30];
	q31	 = q31 + dt * qd31 ;

	qd32  = qd32 + dt * x[31];
	q32	 = q32 + dt * qd32 ;

	qd33  = qd33 + dt * x[32];
	q33	 = q33 + dt * qd33 ;

	qd34  = qd34 + dt * x[33];
	q34	 = q34 + dt * qd34 ;

	qd35  = qd35 + dt * x[34];
	q35	 = q35 + dt * qd35 ;

	qd36  = qd36 + dt * x[35];
	q36	 = q36 + dt * qd36 ;

	qd37  = qd37 + dt * x[36];
	q37	 = q37 + dt * qd37 ;

	qd38  = qd38 + dt * x[37];
	q38	 = q38 + dt * qd38 ;


	 pos_cur = Cvector::Zero(38);

	pos_cur[0] = q1 ;
	pos_cur[1] = q2 ;
	pos_cur[2] = q3 ;
	pos_cur[3] = q4 ;
	pos_cur[4] = q5 ;
	pos_cur[5] = q6 ;
	pos_cur[6] = q7 ;
	pos_cur[7] = q8 ;
	pos_cur[8] = q9 ;
	pos_cur[9] = q10 ;
	pos_cur[10] = q11 ;
	pos_cur[11] = q12 ;
	pos_cur[12] = q13 ;
	pos_cur[13] = q14 ;
	pos_cur[14] = q15 ;
	pos_cur[15] = q16 ;
	pos_cur[16] = q17 ;
	pos_cur[17] = q18 ;
	pos_cur[18] = q19 ;
	pos_cur[19] = q20 ;
	pos_cur[20] = q21 ;
	pos_cur[21] = q22 ;
	pos_cur[22] = q23 ;
	pos_cur[23] = q24 ;
	pos_cur[24] = q25 ;
	pos_cur[25] = q26 ;
	pos_cur[26] = q27 ;
	pos_cur[27] = q28 ;
	pos_cur[28] = q29 ;
	pos_cur[29] = q30 ;
	pos_cur[30] = q31 ;
	pos_cur[31] = q32 ;
	pos_cur[32] = q33 ;
	pos_cur[33] = q34 ;
	pos_cur[34] = q35 ;
	pos_cur[35] = q36 ;
	pos_cur[36] = q37 ;
	pos_cur[37] = q38 ;

	vel_cur = Cvector::Zero(AIR_N_U);

	vel_cur[0] = qd1 ;
	vel_cur[1] = qd2 ;
	vel_cur[2] = qd3 ;
	vel_cur[3] = qd4 ;
	vel_cur[4] = qd5 ;
	vel_cur[5] = qd6 ;
	vel_cur[6] = qd7 ;
	vel_cur[7] = qd8 ;
	vel_cur[8] = qd9 ;
	vel_cur[9] = qd10 ;
	vel_cur[10] = qd11 ;
	vel_cur[11] = qd12 ;
	vel_cur[12] = qd13 ;
	vel_cur[13] = qd14 ;
	vel_cur[14] = qd15 ;
	vel_cur[15] = qd16 ;
	vel_cur[16] = qd17 ;
	vel_cur[17] = qd18 ;
	vel_cur[18] = qd19 ;
	vel_cur[19] = qd20 ;
	vel_cur[20] = qd21 ;
	vel_cur[21] = qd22 ;
	vel_cur[22] = qd23 ;
	vel_cur[23] = qd24 ;
	vel_cur[24] = qd25 ;
	vel_cur[25] = qd26 ;
	vel_cur[26] = qd27 ;
	vel_cur[27] = qd28 ;
	vel_cur[28] = qd29 ;
	vel_cur[29] = qd30 ;
	vel_cur[30] = qd31 ;
	vel_cur[31] = qd32 ;
	vel_cur[32] = qd33 ;
	vel_cur[33] = qd34 ;
	vel_cur[34] = qd35 ;
	vel_cur[35] = qd36 ;
	vel_cur[36] = qd37 ;
	vel_cur[37] = qd38 ;



	iCubModel.set_state(0, pos_cur, vel_cur);
	Cvector acc(38) ;
    end_vel(0)=0;
    end_vel(1)=0;
    end_vel(2)=0;

	end_vel = iCubModel.get_vel(11, Cvector3(0,0,0));

	for (unsigned int i=0 ; i < 38 ; ++i)
	    {
	    	acc[i] = x[i];
	    }

	iCubModel.set_acc(acc);
    end_vel(0)=0;
    end_vel(1)=0;
    end_vel(2)=0;

	end_vel = iCubModel.get_vel(11, Cvector3(0,0,0));



	std::ofstream opt_data ;
	opt_data.open( "opt_data.txt", std::ofstream::out | std::ofstream::app );
	Cvector end_ef = iCubModel.get_pos(11, Cvector3(0,0,0)).transpose() ;

	//opt_data << "Step  = "<< counter << std::endl;
	//opt_data << "q  = "<< pos_cur.transpose() << std::endl;
	//opt_data << "qdot  = "<< vel_cur.transpose() << std::endl;

	//opt_data << "q_dot_elbow  = "<< qd13 << std::endl;

	opt_data << iCubModel.get_pos(11, Cvector3(0,0,0)).transpose() << std::endl ;
	//opt_data << "Final end eff. orientation = "<< iCubModel.get_orient_ang(11).transpose() << std::endl << std::endl;
	//opt_data << "Final COM position = " << iCubModel.get_cm().transpose() << std::endl ;

	if (iCubModel.get_cm()[0] < -0.05 || iCubModel.get_cm()[0] >  0.15)
	{
		opt_data << "X_COM is out of ..." << std::endl;
	}
	if (iCubModel.get_cm()[1] < -0.13 || iCubModel.get_cm()[1]> 0.13)
		{
			opt_data << "Y_COM is out of ..." << std::endl;
		}
	 velocity_n  = iCubModel.get_vel(11, Cvector3(0,0,0));
	//opt_data << "Final Velocity  = " << velocity_n.transpose() << std::endl ;

	velocity_norm = sqrt( velocity_n[0]*velocity_n[0] + velocity_n[1]*velocity_n[1] + velocity_n[2]*velocity_n[2] );


	Cvector3 com = iCubModel.get_cm();
	com_x = com[0];
	com_y = com[1];

    Cvector3 sigma_hdot_1 = Cvector3(0,0,0);
    Cvector3 sigma_pdot = Cvector3(0,0,0);

    for (unsigned int i = 0 ; i < 38 ; ++i)
    {
    	Cvector3 mr2d_i = iCubModel.get_mass(i) * iCubModel.get_acc(i, Cvector3(0,0,0)) ;
    	sigma_pdot = sigma_pdot + mr2d_i ;
    	Cvector3 hdot_i = iCubModel.get_pos(i, Cvector3(0,0,0)).cross(mr2d_i);
    	sigma_hdot_1 = hdot_i + sigma_hdot_1 ;
    }

    zmp_x = (-sigma_hdot_1[1] + (com_x*grav*iCubModel.get_mass()) )/ (sigma_pdot[2] + iCubModel.get_mass() * grav );
    zmp_y = ( sigma_hdot_1[0] + (com_x*grav*iCubModel.get_mass()) )/ (sigma_pdot[2] + iCubModel.get_mass() * grav );

   //opt_data << "Final ZMP position = "<< zmp_x << " , "<< zmp_y << std::endl ;
   /* if (zmp_x < -0.05 || zmp_x >  0.15)
    	{
    		opt_data << "X_ZMP is out of ..." << zmp_x <<std::endl;
    	}
    	if (zmp_y < -0.13 || zmp_y> 0.13)
    		{
    			opt_data << "Y_ZMP is out of ..." << zmp_y<< std::endl;
    		}*/

    Cvector3 error = r_D - end_ef ;
    epsilon = sqrt( error[0]*error[0] + error[1]*error[1] + error[2]*error[2] );

   /* double error = rd_z - end_ef[2] ;
    epsilon = error ;*/
   //opt_data << "error  = "<< epsilon << std::endl << std::endl;

	Cmatrix jac_tr  = iCubModel.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION);
	Cmatrix jac_rot = iCubModel.get_jacob(11, Cvector3(0,0,0), CT_ROTATION);

	//Cvector3 new_vel = (jac_tr*vel_cur);

	//std::cout <<  new_vel<< std::endl;

	Cmatrix jj = jac_tr.transpose();

    jj = jj * jac_tr ;

  

} 

