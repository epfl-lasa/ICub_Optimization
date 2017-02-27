/*!
 * \file Description.hpp
 *
 * \author Salman Faraji
 * \date 10.2.2015
 *
 * Describes the indexing as well as geometry of the robot's states
 */

# pragma once

    /* Sizes */
    //! Total number of states (including additional quaternion 4th element)
    #define AIR_N_Q 39
    //! Total number of state derivatives
    #define AIR_N_U 38
    #define AIR_N_STATE (AIR_N_Q+AIR_N_U)

    #define AIR_N_JOINTS 39
    #define AIR_N_BODY 39
 #define AIR_BODY_L_hand_base 11

/*

     Indices for sdfast state
    #define S_AIR_ROOT_X 0
    #define S_AIR_ROOT_Y 1
    #define S_AIR_ROOT_Z 2
    #define S_AIR_ROOT_Q0 3
    #define S_AIR_ROOT_Q1 4
    #define S_AIR_ROOT_Q2 5
    #define S_AIR_ROOT_Q3 29

    #define S_AIR_TORSO_X 6
    #define S_AIR_TORSO_Y 7
    #define S_AIR_TORSO_Z 8

    #define S_AIR_L_LEG_HIP_Y 9
    #define S_AIR_L_LEG_HIP_X 10
    #define S_AIR_L_LEG_HIP_Z 11
    #define S_AIR_L_LEG_KNEE_Y 12
    #define S_AIR_L_LEG_FOOT_X 13
    #define S_AIR_L_LEG_FOOT_Y 14

    #define S_AIR_R_LEG_HIP_Y 15
    #define S_AIR_R_LEG_HIP_X 16
    #define S_AIR_R_LEG_HIP_Z 17
    #define S_AIR_R_LEG_KNEE_Y 18
    #define S_AIR_R_LEG_FOOT_X 19
    #define S_AIR_R_LEG_FOOT_Y 20

    #define S_AIR_L_ARM_SHOULDER_Y 21
    #define S_AIR_L_ARM_SHOULDER_X 22
    #define S_AIR_L_ARM_SHOULDER_Z 23
    #define S_AIR_L_ARM_ELBOW_Y 24

    #define S_AIR_R_ARM_SHOULDER_Y 25
    #define S_AIR_R_ARM_SHOULDER_X 26
    #define S_AIR_R_ARM_SHOULDER_Z 27
    #define S_AIR_R_ARM_ELBOW_Y 28

    // velocities
    #define S_AIR_ROOT_XD 0
    #define S_AIR_ROOT_YD 1
    #define S_AIR_ROOT_ZD 2
    #define S_AIR_ROOT_Q0D 3
    #define S_AIR_ROOT_Q1D 4
    #define S_AIR_ROOT_Q2D 5

    #define S_AIR_TORSO_XD 6
    #define S_AIR_TORSO_YD 7
    #define S_AIR_TORSO_ZD 8

    #define S_AIR_L_LEG_HIP_YD 9
    #define S_AIR_L_LEG_HIP_XD 10
    #define S_AIR_L_LEG_HIP_ZD 11
    #define S_AIR_L_LEG_KNEE_YD 12
    #define S_AIR_L_LEG_FOOT_XD 13
    #define S_AIR_L_LEG_FOOT_YD 14

    #define S_AIR_R_LEG_HIP_YD 15
    #define S_AIR_R_LEG_HIP_XD 16
    #define S_AIR_R_LEG_HIP_ZD 17
    #define S_AIR_R_LEG_KNEE_YD 18
    #define S_AIR_R_LEG_FOOT_XD 19
    #define S_AIR_R_LEG_FOOT_YD 20

    #define S_AIR_L_ARM_SHOULDER_YD 21
    #define S_AIR_L_ARM_SHOULDER_XD 22
    #define S_AIR_L_ARM_SHOULDER_ZD 23
    #define S_AIR_L_ARM_ELBOW_YD 24

    #define S_AIR_R_ARM_SHOULDER_YD 25
    #define S_AIR_R_ARM_SHOULDER_XD 26
    #define S_AIR_R_ARM_SHOULDER_ZD 27
    #define S_AIR_R_ARM_ELBOW_YD 28


    ************

    // Body index definitions (for sdpointf, etc.)

    #define AIR_BODY_ROBOT 0
    #define AIR_BODY_TORSO_X 1
    #define AIR_BODY_TORSO_Y 2
    #define AIR_BODY_TORSO_Z 3

    #define AIR_BODY_L_LEG_HIP_Y 4
    #define AIR_BODY_L_LEG_HIP_X 5
    #define AIR_BODY_L_LEG_HIP_Z 6
    #define AIR_BODY_L_LEG_KNEE_Y 7
    #define AIR_BODY_L_LEG_FOOT_X 8
    #define AIR_BODY_L_LEG_FOOT_Y 9

    #define AIR_BODY_R_LEG_HIP_Y 10
    #define AIR_BODY_R_LEG_HIP_X 11
    #define AIR_BODY_R_LEG_HIP_Z 12
    #define AIR_BODY_R_LEG_KNEE_Y 13
    #define AIR_BODY_R_LEG_FOOT_X 14
    #define AIR_BODY_R_LEG_FOOT_Y 15

    #define AIR_BODY_L_ARM_SHOULDER_Y 16
    #define AIR_BODY_L_ARM_SHOULDER_X 17
    #define AIR_BODY_L_ARM_SHOULDER_Z 18
    #define AIR_BODY_L_ARM_ELBOW_Y 19

    #define AIR_BODY_R_ARM_SHOULDER_Y 20
    #define AIR_BODY_R_ARM_SHOULDER_X 21
    #define AIR_BODY_R_ARM_SHOULDER_Z 22
    #define AIR_BODY_R_ARM_ELBOW_Y 23
    // #define AIR_BODY_LOOP 24
    #define AIR_N_BODY 24

    // Joint index definitions (for sdhinget, etc.)
    #define AIR_JOINT_ROOT 0

    #define AIR_JOINT_TORSO_X 1
    #define AIR_JOINT_TORSO_Y 2
    #define AIR_JOINT_TORSO_Z 3

    #define AIR_JOINT_L_LEG_HIP_Y 4
    #define AIR_JOINT_L_LEG_HIP_X 5
    #define AIR_JOINT_L_LEG_HIP_Z 6
    #define AIR_JOINT_L_LEG_KNEE_Y 7
    #define AIR_JOINT_L_LEG_FOOT_X 8
    #define AIR_JOINT_L_LEG_FOOT_Y 9

    #define AIR_JOINT_R_LEG_HIP_Y 10
    #define AIR_JOINT_R_LEG_HIP_X 11
    #define AIR_JOINT_R_LEG_HIP_Z 12
    #define AIR_JOINT_R_LEG_KNEE_Y 13
    #define AIR_JOINT_R_LEG_FOOT_X 14
    #define AIR_JOINT_R_LEG_FOOT_Y 15

    #define AIR_JOINT_L_ARM_SHOULDER_Y 16
    #define AIR_JOINT_L_ARM_SHOULDER_X 17
    #define AIR_JOINT_L_ARM_SHOULDER_Z 18
    #define AIR_JOINT_L_ARM_ELBOW_Y 19

    #define AIR_JOINT_R_ARM_SHOULDER_Y 20
    #define AIR_JOINT_R_ARM_SHOULDER_X 21
    #define AIR_JOINT_R_ARM_SHOULDER_Z 22
    #define AIR_JOINT_R_ARM_ELBOW_Y 23

    #define AIR_N_JOINTS 24

    ************
    //! Maximum change of joint torques over timesteps
    #define max_delta_tau 50
    //! Maximum actuator limit
    #define act_lim 30.0

    //! Safety around the foot surface
    #define margin 0.0
    //! The half-width of the foot (shoe)
    #define wf 0.03
    //! The half-length of the foot (shoe)
    #define lf 0.07
    //! Foot length
    #define foot_lenght 0.19
    //! Foot width
    #define foot_width  0.09

    //! The body index of the left foot point
    #define body_l_foot 	AIR_BODY_L_LEG_FOOT_Y
    //! The link-frame offset of the left foot point
    #define offset_l_foot 	Cvector3(-0.01+0.035,-0.009,-0.057)
    //! The body index of the right foot point
    #define body_r_foot 	AIR_BODY_R_LEG_FOOT_Y
    //! The link-frame offset of the right foot point
    #define offset_r_foot 	Cvector3(-0.01+0.035, 0.009,-0.057)
    //! The body index of the left hand point
    #define body_l_hand 	AIR_BODY_L_ARM_ELBOW_Y
    //! The link-frame offset of the left hand point
    #define offset_l_hand 	Cvector3(-0.01, 0.0, -0.163)
    //! The body index of the right hand point
    #define body_r_hand 	AIR_BODY_R_ARM_ELBOW_Y
    //! The link-frame offset of the right hand point
    #define offset_r_hand 	Cvector3(-0.01, 0.0, -0.163)
    //! The body index of the base point
    #define body_base 	    	AIR_BODY_ROBOT
    //! The link-frame offset of the base point
    #define offset_base 	Cvector3(0.0, 0.0, 0.0)
*/

