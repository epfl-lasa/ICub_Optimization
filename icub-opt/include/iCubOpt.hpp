#pragma once 
#include "snoptStructure.hpp"
#include "Model.hpp"
 
class iCubOpt
{ 
public: 
    iCubOpt(){};//{init();};
    ~iCubOpt() {};
    void init(); 
    void update(double x[]);  //Neda

    vars W_FULL_vars;
    void W_FULL_init();
    void W_FULL_delete();
    void W_FULL_shot(const double sx[], double sF[], double sG[], double sH[], bool kin);

    vars W_SIMPLE_vars;
    void W_SIMPLE_init();
    void W_SIMPLE_delete();
    void W_SIMPLE_shot(const double sx[], double sF[], double sG[], double sH[], bool kin);

    vars W_STATIC_vars;
    void W_STATIC_init();
    void W_STATIC_update(double x[]);

    void W_STATIC_delete();
    void W_STATIC_shot(const double sx[], double sF[],  double sG[]);

    double velocity_norm;
    int counter ;
    double dt = 0.001 ;
    double com_x ;
    double com_y ;
    double rd_x = 0.166843 ;
    double rd_y = 0.214419 ;
    double rd_z = 0.498204 ;
    double grav = -9.8 ;
    double epsilon ;
    Cvector3 r_D = Cvector3(rd_x,rd_y,rd_z);
    Cvector3 end_pos ;
    Cvector3 end_vel ;
    double dir_norm;
    double zmp_x ;
    double zmp_y ;
    Cvector pos_cur ;
    Cvector vel_cur;
    Cvector3 dir_cons;
    Cvector3 velocity_n;

    double j1_1 ;
    double j2_1 ;
    double j3_1 ;
    double j4_1 ;
    double j5_1 ;
    double j6_1 ;
    double j7_1 ;
    double j8_1 ;
    double j9_1 ;
    double j10_1 ;
    double j11_1 ;
    double j12_1 ;
    double j13_1 ;
    double j14_1 ;
    double j15_1 ;
    double j16_1 ;

    double j1_2 ;
    double j2_2 ;
    double j3_2 ;
    double j4_2 ;
    double j5_2 ;
    double j6_2 ;
    double j7_2 ;
    double j8_2 ;
    double j9_2 ;
    double j10_2 ;
    double j11_2 ;
    double j12_2 ;
    double j13_2 ;
    double j14_2 ;
    double j15_2 ;
    double j16_2 ;

    double j1_3 ;
    double j2_3 ;
    double j3_3 ;
    double j4_3 ;
    double j5_3 ;
    double j6_3 ;
    double j7_3 ;
    double j8_3 ;
    double j9_3 ;
    double j10_3 ;
    double j11_3 ;
    double j12_3 ;
    double j13_3 ;
    double j14_3 ;
    double j15_3 ;
    double j16_3 ;

    double j1_4 ;
    double j2_4 ;
    double j3_4 ;
    double j4_4 ;
    double j5_4 ;
    double j6_4 ;
    double j7_4 ;
    double j8_4 ;
    double j9_4 ;
    double j10_4 ;
    double j11_4 ;
    double j12_4 ;
    double j13_4 ;
    double j14_4 ;
    double j15_4 ;
    double j16_4 ;


    double j1_5 ;
    double j2_5 ;
    double j3_5 ;
    double j4_5 ;
    double j5_5 ;
    double j6_5 ;
    double j7_5 ;
    double j8_5 ;
    double j9_5 ;
    double j10_5 ;
    double j11_5 ;
    double j12_5 ;
    double j13_5 ;
    double j14_5 ;
    double j15_5 ;
    double j16_5 ;

    double j1_6 ;
    double j2_6 ;
    double j3_6 ;
    double j4_6 ;
    double j5_6 ;
    double j6_6 ;
    double j7_6 ;
    double j8_6 ;
    double j9_6 ;
    double j10_6 ;
    double j11_6 ;
    double j12_6 ;
    double j13_6 ;
    double j14_6 ;
    double j15_6 ;
    double j16_6 ;

    double j1_7 ;
    double j2_7 ;
    double j3_7 ;
    double j4_7 ;
    double j5_7 ;
    double j6_7 ;
    double j7_7 ;
    double j8_7 ;
    double j9_7 ;
    double j10_7 ;
    double j11_7 ;
    double j12_7 ;
    double j13_7 ;
    double j14_7 ;
    double j15_7 ;
    double j16_7 ;

    double j1_8 ;
    double j2_8 ;
    double j3_8 ;
    double j4_8 ;
    double j5_8 ;
    double j6_8 ;
    double j7_8 ;
    double j8_8 ;
    double j9_8 ;
    double j10_8 ;
    double j11_8 ;
    double j12_8 ;
    double j13_8 ;
    double j14_8 ;
    double j15_8 ;
    double j16_8 ;

    double j1_9 ;
    double j2_9 ;
    double j3_9 ;
    double j4_9 ;
    double j5_9 ;
    double j6_9 ;
    double j7_9 ;
    double j8_9 ;
    double j9_9 ;
    double j10_9 ;
    double j11_9 ;
    double j12_9 ;
    double j13_9 ;
    double j14_9 ;
    double j15_9 ;
    double j16_9 ;

    double j1_10 ;
    double j2_10 ;
    double j3_10 ;
    double j4_10 ;
    double j5_10 ;
    double j6_10 ;
    double j7_10 ;
    double j8_10 ;
    double j9_10 ;
    double j10_10 ;
    double j11_10 ;
    double j12_10 ;
    double j13_10 ;
    double j14_10 ;
    double j15_10 ;
    double j16_10 ;

    double j1_11 ;
    double j2_11 ;
    double j3_11 ;
    double j4_11 ;
    double j5_11 ;
    double j6_11 ;
    double j7_11 ;
    double j8_11 ;
    double j9_11 ;
    double j10_11 ;
    double j11_11 ;
    double j12_11 ;
    double j13_11 ;
    double j14_11 ;
    double j15_11 ;
    double j16_11 ;

    double j1_12 ;
    double j2_12 ;
    double j3_12 ;
    double j4_12 ;
    double j5_12 ;
    double j6_12 ;
    double j7_12 ;
    double j8_12 ;
    double j9_12 ;
    double j10_12 ;
    double j11_12 ;
    double j12_12 ;
    double j13_12 ;
    double j14_12 ;
    double j15_12 ;
    double j16_12 ;

    double j1_13 ;
    double j2_13 ;
    double j3_13 ;
    double j4_13 ;
    double j5_13 ;
    double j6_13 ;
    double j7_13 ;
    double j8_13 ;
    double j9_13 ;
    double j10_13 ;
    double j11_13 ;
    double j12_13 ;
    double j13_13 ;
    double j14_13 ;
    double j15_13 ;
    double j16_13 ;

    double j1_14 ;
    double j2_14 ;
    double j3_14 ;
    double j4_14 ;
    double j5_14 ;
    double j6_14 ;
    double j7_14 ;
    double j8_14 ;
    double j9_14 ;
    double j10_14 ;
    double j11_14 ;
    double j12_14 ;
    double j13_14 ;
    double j14_14 ;
    double j15_14 ;
    double j16_14 ;

    double j1_15 ;
    double j2_15 ;
    double j3_15 ;
    double j4_15 ;
    double j5_15 ;
    double j6_15 ;
    double j7_15 ;
    double j8_15 ;
    double j9_15 ;
    double j10_15 ;
    double j11_15 ;
    double j12_15 ;
    double j13_15 ;
    double j14_15 ;
    double j15_15 ;
    double j16_15 ;

    double j1_16 ;
    double j2_16 ;
    double j3_16 ;
    double j4_16 ;
    double j5_16 ;
    double j6_16 ;
    double j7_16 ;
    double j8_16 ;
    double j9_16 ;
    double j10_16 ;
    double j11_16 ;
    double j12_16 ;
    double j13_16 ;
    double j14_16 ;
    double j15_16 ;
    double j16_16 ;

    double q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,q17,q18,q19,q20,q21,q22,q23,q24,q25,q26,q27,q28,q29,q30,q31,q32,q33,q34,q35,q36,q37,q38;
    double qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14,qd15,qd16,qd17,qd18,qd19,qd20;
    double qd21,qd22,qd23,qd24,qd25,qd26,qd27,qd28,qd29,qd30,qd31,qd32,qd33,qd34,qd35,qd36,qd37,qd38;
    Model iCubModel_1;



protected:
    Model iCubModel;


}; 
