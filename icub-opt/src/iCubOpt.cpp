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


    j1_1 = jj(0,0) ;   j1_2 = jj(0,1) ;		j1_3 = jj(0,2) ;	j1_4 = jj(0,3) ;	j1_5 = jj(0,4) ;	j1_6 = jj(0,5) ;	j1_7 = jj(0,6) ;	j1_8 = jj(0,7) ;
    j2_1 = jj(1,0) ;   j2_2 = jj(1,1) ;		j2_3 = jj(1,2) ;	j2_4 = jj(1,3) ;	j2_5 = jj(1,4) ;	j2_6 = jj(1,5) ;	j2_7 = jj(1,6) ;	j2_8 = jj(1,7) ;
    j3_1 = jj(2,0) ;   j3_2 = jj(2,1) ;		j3_3 = jj(2,2) ;	j3_4 = jj(2,3) ;	j3_5 = jj(2,4) ;	j3_6 = jj(2,5) ;	j3_7 = jj(2,6) ;	j3_8 = jj(2,7) ;
    j4_1 = jj(3,0) ;   j4_2 = jj(3,1) ;		j4_3 = jj(3,2) ;	j4_4 = jj(3,3) ;	j4_5 = jj(3,4) ;	j4_6 = jj(3,5) ;	j4_7 = jj(3,6) ;	j4_8 = jj(3,7) ;
    j5_1 = jj(4,0) ;   j5_2 = jj(4,1) ;		j5_3 = jj(4,2) ;	j5_4 = jj(4,3) ;	j5_5 = jj(4,4) ;	j5_6 = jj(4,5) ;	j5_7 = jj(4,6) ;	j5_8 = jj(4,7) ;
    j6_1 = jj(5,0) ;   j6_2 = jj(5,1) ;		j6_3 = jj(5,2) ;	j6_4 = jj(5,3) ;	j6_5 = jj(5,4) ;	j6_6 = jj(5,5) ;	j6_7 = jj(5,6) ;	j6_8 = jj(5,7) ;
    j7_1 = jj(6,0) ;   j7_2 = jj(6,1) ;		j7_3 = jj(6,2) ;	j7_4 = jj(6,3) ;	j7_5 = jj(6,4) ;	j7_6 = jj(6,5) ;	j7_7 = jj(6,6) ;	j7_8 = jj(6,7) ;
    j8_1 = jj(7,0) ;   j8_2 = jj(7,1) ;		j8_3 = jj(7,2) ;	j8_4 = jj(7,3) ;	j8_5 = jj(7,4) ;	j8_6 = jj(7,5) ;	j8_7 = jj(7,6) ;	j8_8 = jj(7,7) ;
    j9_1 = jj(8,0) ;   j9_2 = jj(8,1) ;		j9_3 = jj(8,2) ;	j9_4 = jj(8,3) ;	j9_5 = jj(8,4) ;	j9_6 = jj(8,5) ;	j9_7 = jj(8,6) ;	j9_8 = jj(8,7) ;
    j10_1 = jj(9,0) ;  j10_2 = jj(9,1) ;	j10_3 = jj(9,2) ;	j10_4 = jj(9,3) ;	j10_5 = jj(9,4) ;	j10_6 = jj(9,5) ;	j10_7 = jj(9,6) ;	j10_8 = jj(9,7) ;
    j11_1 = jj(10,0) ; j11_2 = jj(10,1) ;	j11_3 = jj(10,2) ;	j11_4 = jj(10,3) ;	j11_5 = jj(10,4) ;	j11_6 = jj(10,5) ;	j11_7 = jj(10,6) ;	j11_8 = jj(10,7) ;
    j12_1 = jj(11,0) ; j12_2 = jj(11,1) ;	j12_3 = jj(11,2) ;	j12_4 = jj(11,3) ;	j12_5 = jj(11,4) ;	j12_6 = jj(11,5) ;	j12_7 = jj(11,6) ;	j12_8 = jj(11,7) ;
    j13_1 = jj(12,0) ; j13_2 = jj(12,1) ;	j13_3 = jj(12,2) ;	j13_4 = jj(12,3) ;	j13_5 = jj(12,4) ;	j13_6 = jj(12,5) ;	j13_7 = jj(12,6) ;	j13_8 = jj(12,7) ;
    j14_1 = jj(13,0) ; j14_2 = jj(13,1) ;	j14_3 = jj(13,2) ;	j14_4 = jj(13,3) ;	j14_5 = jj(13,4) ;	j14_6 = jj(13,5) ;	j14_7 = jj(13,6) ;	j14_8 = jj(13,7) ;
    j15_1 = jj(14,0) ; j15_2 = jj(14,1) ;	j15_3 = jj(14,2) ;	j15_4 = jj(14,3) ;	j15_5 = jj(14,4) ;	j15_6 = jj(14,5) ;	j15_7 = jj(14,6) ;	j15_8 = jj(14,7) ;
    j16_1 = jj(15,0) ; j16_2 = jj(15,1) ;	j16_3 = jj(15,2) ;	j16_4 = jj(15,3) ;	j16_5 = jj(15,4) ;	j16_6 = jj(15,5) ;	j16_7 = jj(15,6) ;	j16_8 = jj(15,7) ;


    j1_9 = jj(0,8) ;	j1_10 = jj(0,9) ;	j1_11 = jj(0,10) ;	j1_12 = jj(0,11) ;	j1_13 = jj(0,12) ;	j1_14 = jj(0,13) ;	j1_15 = jj(0,14) ;	j1_16 = jj(0,15) ;
    j2_9 = jj(1,8) ; 	j2_10 = jj(1,9) ;	j2_11 = jj(1,10) ;	j2_12 = jj(1,11) ;	j2_13 = jj(1,12) ;	j2_14 = jj(1,13) ;	j2_15 = jj(1,14) ;	j2_16 = jj(1,15) ;
    j3_9 = jj(2,8) ; 	j3_10 = jj(2,9) ;	j3_11 = jj(2,10) ;	j3_12 = jj(2,11) ;	j3_13 = jj(2,12) ;	j3_14 = jj(2,13) ;	j3_15 = jj(2,14) ;	j3_16 = jj(2,15) ;
    j4_9 = jj(3,8) ; 	j4_10 = jj(3,9) ;	j4_11 = jj(3,10) ;	j4_12 = jj(3,11) ;	j4_13 = jj(3,12) ;	j4_14 = jj(3,13) ;	j4_15 = jj(3,14) ;	j4_16 = jj(3,15) ;
    j5_9 = jj(4,8) ; 	j5_10 = jj(4,9) ;	j5_11 = jj(4,10) ;	j5_12 = jj(4,11) ;	j5_13 = jj(4,12) ;	j5_14 = jj(4,13) ;	j5_15 = jj(4,14) ;	j5_16 = jj(4,15) ;
    j6_9 = jj(5,8) ; 	j6_10 = jj(5,9) ;	j6_11 = jj(5,10) ;	j6_12 = jj(5,11) ;	j6_13 = jj(5,12) ;	j6_14 = jj(5,13) ;	j6_15 = jj(5,14) ;	j6_16 = jj(5,15) ;
    j7_9 = jj(6,8) ; 	j7_10 = jj(6,9) ;	j7_11 = jj(6,10) ;	j7_12 = jj(6,11) ;	j7_13 = jj(6,12) ;	j7_14 = jj(6,13) ;	j7_15 = jj(6,14) ;	j7_16 = jj(6,15) ;
    j8_9 = jj(7,8) ; 	j8_10 = jj(7,9) ;	j8_11 = jj(7,10) ;	j8_12 = jj(7,11) ;	j8_13 = jj(7,12) ;	j8_14 = jj(7,13) ;	j8_15 = jj(7,14) ;	j8_16 = jj(7,15) ;
    j9_9 = jj(8,8) ; 	j9_10 = jj(8,9) ;	j9_11 = jj(8,10) ;	j9_12 = jj(8,11) ;	j9_13 = jj(8,12) ;	j9_14 = jj(8,13) ;	j9_15 = jj(8,14) ;	j9_16 = jj(8,15) ;
    j10_9 = jj(9,8) ; 	j10_10 = jj(9,9) ;  j10_11 = jj(9,10) ; j10_12 = jj(9,11) ; j10_13 = jj(9,12) ; j10_14 = jj(9,13) ; j10_15 = jj(9,14) ;j10_16 = jj(9,15) ;
    j11_9 = jj(10,8) ;  j11_10 = jj(10,9) ; j11_11 = jj(10,10) ;j11_12 = jj(10,11) ;j11_13 = jj(10,12) ;j11_14 = jj(10,13) ;j11_15 = jj(10,14) ;j11_16 = jj(10,15) ;
    j12_9 = jj(11,8) ;  j12_10 = jj(11,9) ; j12_11 = jj(11,10) ;j12_12 = jj(11,11) ;j12_13 = jj(11,12) ;j12_14 = jj(11,13) ;j12_15 = jj(11,14) ;j12_16 = jj(11,15) ;
    j13_9 = jj(12,8) ;  j13_10 = jj(12,9) ; j13_11 = jj(12,10) ;j13_12 = jj(12,11) ;j13_13 = jj(12,12) ;j13_14 = jj(12,13) ;j13_15 = jj(12,14) ;j13_16 = jj(12,15) ;
    j14_9 = jj(13,8) ;  j14_10 = jj(13,9) ; j14_11 = jj(13,10) ;j14_12 = jj(13,11) ;j14_13 = jj(13,12) ;j14_14 = jj(13,13) ;j14_15 = jj(13,14) ;j14_16 = jj(13,15) ;
    j15_9 = jj(14,8) ;  j15_10 = jj(14,9) ; j15_11 = jj(14,10) ;j15_12 = jj(14,11) ;j15_13 = jj(14,12) ;j15_14 = jj(14,13) ;j15_15 = jj(14,14) ;j15_16 = jj(14,15) ;
    j16_9 = jj(15,8) ;  j16_10 = jj(15,9) ; j16_11 = jj(15,10) ;j16_12 = jj(15,11) ;j16_13 = jj(15,12) ;j16_14 = jj(15,13) ;j16_15 = jj(15,14) ;j16_16 = jj(15,15) ;


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

    j1_1 = jj(0,0) ;   j1_2 = jj(0,1) ;		j1_3 = jj(0,2) ;	j1_4 = jj(0,3) ;	j1_5 = jj(0,4) ;	j1_6 = jj(0,5) ;	j1_7 = jj(0,6) ;	j1_8 = jj(0,7) ;
    j2_1 = jj(1,0) ;   j2_2 = jj(1,1) ;		j2_3 = jj(1,2) ;	j2_4 = jj(1,3) ;	j2_5 = jj(1,4) ;	j2_6 = jj(1,5) ;	j2_7 = jj(1,6) ;	j2_8 = jj(1,7) ;
    j3_1 = jj(2,0) ;   j3_2 = jj(2,1) ;		j3_3 = jj(2,2) ;	j3_4 = jj(2,3) ;	j3_5 = jj(2,4) ;	j3_6 = jj(2,5) ;	j3_7 = jj(2,6) ;	j3_8 = jj(2,7) ;
    j4_1 = jj(3,0) ;   j4_2 = jj(3,1) ;		j4_3 = jj(3,2) ;	j4_4 = jj(3,3) ;	j4_5 = jj(3,4) ;	j4_6 = jj(3,5) ;	j4_7 = jj(3,6) ;	j4_8 = jj(3,7) ;
    j5_1 = jj(4,0) ;   j5_2 = jj(4,1) ;		j5_3 = jj(4,2) ;	j5_4 = jj(4,3) ;	j5_5 = jj(4,4) ;	j5_6 = jj(4,5) ;	j5_7 = jj(4,6) ;	j5_8 = jj(4,7) ;
    j6_1 = jj(5,0) ;   j6_2 = jj(5,1) ;		j6_3 = jj(5,2) ;	j6_4 = jj(5,3) ;	j6_5 = jj(5,4) ;	j6_6 = jj(5,5) ;	j6_7 = jj(5,6) ;	j6_8 = jj(5,7) ;
    j7_1 = jj(6,0) ;   j7_2 = jj(6,1) ;		j7_3 = jj(6,2) ;	j7_4 = jj(6,3) ;	j7_5 = jj(6,4) ;	j7_6 = jj(6,5) ;	j7_7 = jj(6,6) ;	j7_8 = jj(6,7) ;
    j8_1 = jj(7,0) ;   j8_2 = jj(7,1) ;		j8_3 = jj(7,2) ;	j8_4 = jj(7,3) ;	j8_5 = jj(7,4) ;	j8_6 = jj(7,5) ;	j8_7 = jj(7,6) ;	j8_8 = jj(7,7) ;
    j9_1 = jj(8,0) ;   j9_2 = jj(8,1) ;		j9_3 = jj(8,2) ;	j9_4 = jj(8,3) ;	j9_5 = jj(8,4) ;	j9_6 = jj(8,5) ;	j9_7 = jj(8,6) ;	j9_8 = jj(8,7) ;
    j10_1 = jj(9,0) ;  j10_2 = jj(9,1) ;	j10_3 = jj(9,2) ;	j10_4 = jj(9,3) ;	j10_5 = jj(9,4) ;	j10_6 = jj(9,5) ;	j10_7 = jj(9,6) ;	j10_8 = jj(9,7) ;
    j11_1 = jj(10,0) ; j11_2 = jj(10,1) ;	j11_3 = jj(10,2) ;	j11_4 = jj(10,3) ;	j11_5 = jj(10,4) ;	j11_6 = jj(10,5) ;	j11_7 = jj(10,6) ;	j11_8 = jj(10,7) ;
    j12_1 = jj(11,0) ; j12_2 = jj(11,1) ;	j12_3 = jj(11,2) ;	j12_4 = jj(11,3) ;	j12_5 = jj(11,4) ;	j12_6 = jj(11,5) ;	j12_7 = jj(11,6) ;	j12_8 = jj(11,7) ;
    j13_1 = jj(12,0) ; j13_2 = jj(12,1) ;	j13_3 = jj(12,2) ;	j13_4 = jj(12,3) ;	j13_5 = jj(12,4) ;	j13_6 = jj(12,5) ;	j13_7 = jj(12,6) ;	j13_8 = jj(12,7) ;
    j14_1 = jj(13,0) ; j14_2 = jj(13,1) ;	j14_3 = jj(13,2) ;	j14_4 = jj(13,3) ;	j14_5 = jj(13,4) ;	j14_6 = jj(13,5) ;	j14_7 = jj(13,6) ;	j14_8 = jj(13,7) ;
    j15_1 = jj(14,0) ; j15_2 = jj(14,1) ;	j15_3 = jj(14,2) ;	j15_4 = jj(14,3) ;	j15_5 = jj(14,4) ;	j15_6 = jj(14,5) ;	j15_7 = jj(14,6) ;	j15_8 = jj(14,7) ;
    j16_1 = jj(15,0) ; j16_2 = jj(15,1) ;	j16_3 = jj(15,2) ;	j16_4 = jj(15,3) ;	j16_5 = jj(15,4) ;	j16_6 = jj(15,5) ;	j16_7 = jj(15,6) ;	j16_8 = jj(15,7) ;


    j1_9 = jj(0,8) ;	j1_10 = jj(0,9) ;	j1_11 = jj(0,10) ;	j1_12 = jj(0,11) ;	j1_13 = jj(0,12) ;	j1_14 = jj(0,13) ;	j1_15 = jj(0,14) ;	j1_16 = jj(0,15) ;
    j2_9 = jj(1,8) ; 	j2_10 = jj(1,9) ;	j2_11 = jj(1,10) ;	j2_12 = jj(1,11) ;	j2_13 = jj(1,12) ;	j2_14 = jj(1,13) ;	j2_15 = jj(1,14) ;	j2_16 = jj(1,15) ;
    j3_9 = jj(2,8) ; 	j3_10 = jj(2,9) ;	j3_11 = jj(2,10) ;	j3_12 = jj(2,11) ;	j3_13 = jj(2,12) ;	j3_14 = jj(2,13) ;	j3_15 = jj(2,14) ;	j3_16 = jj(2,15) ;
    j4_9 = jj(3,8) ; 	j4_10 = jj(3,9) ;	j4_11 = jj(3,10) ;	j4_12 = jj(3,11) ;	j4_13 = jj(3,12) ;	j4_14 = jj(3,13) ;	j4_15 = jj(3,14) ;	j4_16 = jj(3,15) ;
    j5_9 = jj(4,8) ; 	j5_10 = jj(4,9) ;	j5_11 = jj(4,10) ;	j5_12 = jj(4,11) ;	j5_13 = jj(4,12) ;	j5_14 = jj(4,13) ;	j5_15 = jj(4,14) ;	j5_16 = jj(4,15) ;
    j6_9 = jj(5,8) ; 	j6_10 = jj(5,9) ;	j6_11 = jj(5,10) ;	j6_12 = jj(5,11) ;	j6_13 = jj(5,12) ;	j6_14 = jj(5,13) ;	j6_15 = jj(5,14) ;	j6_16 = jj(5,15) ;
    j7_9 = jj(6,8) ; 	j7_10 = jj(6,9) ;	j7_11 = jj(6,10) ;	j7_12 = jj(6,11) ;	j7_13 = jj(6,12) ;	j7_14 = jj(6,13) ;	j7_15 = jj(6,14) ;	j7_16 = jj(6,15) ;
    j8_9 = jj(7,8) ; 	j8_10 = jj(7,9) ;	j8_11 = jj(7,10) ;	j8_12 = jj(7,11) ;	j8_13 = jj(7,12) ;	j8_14 = jj(7,13) ;	j8_15 = jj(7,14) ;	j8_16 = jj(7,15) ;
    j9_9 = jj(8,8) ; 	j9_10 = jj(8,9) ;	j9_11 = jj(8,10) ;	j9_12 = jj(8,11) ;	j9_13 = jj(8,12) ;	j9_14 = jj(8,13) ;	j9_15 = jj(8,14) ;	j9_16 = jj(8,15) ;
    j10_9 = jj(9,8) ; 	j10_10 = jj(9,9) ;  j10_11 = jj(9,10) ; j10_12 = jj(9,11) ; j10_13 = jj(9,12) ; j10_14 = jj(9,13) ; j10_15 = jj(9,14) ;j10_16 = jj(9,15) ;
    j11_9 = jj(10,8) ;  j11_10 = jj(10,9) ; j11_11 = jj(10,10) ;j11_12 = jj(10,11) ;j11_13 = jj(10,12) ;j11_14 = jj(10,13) ;j11_15 = jj(10,14) ;j11_16 = jj(10,15) ;
    j12_9 = jj(11,8) ;  j12_10 = jj(11,9) ; j12_11 = jj(11,10) ;j12_12 = jj(11,11) ;j12_13 = jj(11,12) ;j12_14 = jj(11,13) ;j12_15 = jj(11,14) ;j12_16 = jj(11,15) ;
    j13_9 = jj(12,8) ;  j13_10 = jj(12,9) ; j13_11 = jj(12,10) ;j13_12 = jj(12,11) ;j13_13 = jj(12,12) ;j13_14 = jj(12,13) ;j13_15 = jj(12,14) ;j13_16 = jj(12,15) ;
    j14_9 = jj(13,8) ;  j14_10 = jj(13,9) ; j14_11 = jj(13,10) ;j14_12 = jj(13,11) ;j14_13 = jj(13,12) ;j14_14 = jj(13,13) ;j14_15 = jj(13,14) ;j14_16 = jj(13,15) ;
    j15_9 = jj(14,8) ;  j15_10 = jj(14,9) ; j15_11 = jj(14,10) ;j15_12 = jj(14,11) ;j15_13 = jj(14,12) ;j15_14 = jj(14,13) ;j15_15 = jj(14,14) ;j15_16 = jj(14,15) ;
    j16_9 = jj(15,8) ;  j16_10 = jj(15,9) ; j16_11 = jj(15,10) ;j16_12 = jj(15,11) ;j16_13 = jj(15,12) ;j16_14 = jj(15,13) ;j16_15 = jj(15,14) ;j16_16 = jj(15,15) ;


} 

