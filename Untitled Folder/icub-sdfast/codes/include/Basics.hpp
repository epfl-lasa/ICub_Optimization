/*!
 * \file Basics.hpp
 *
 * \author Salman Faraji
 * \date 10.2.2015
 *
 * The basic definitions, file inclusions and math functions
 */

#pragma once

// C++ standard headers
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <fstream>
#include <vector>
#include "vector"
#include <unistd.h>
#include <string>
#include <time.h>
#include <iostream>

// Eigen headers
#define MY_EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(3, 0, " ", " ", " ", " ", "\t", "\n")
#define EIGEN_MATRIX_IO_FORMAT Eigen::IOFormat()
#include </usr/include/eigen3/Eigen/Dense>

//#define Cmatrix Eigen::MatrixXd
//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
//! The class of Cvector abreviated from Eigen VectorXd
#define Cvector Eigen::VectorXd
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d
#define Cvector4 Eigen::Vector4d
#define zero_v3 Cvector3(0.0,0.0,0.0)

# define NUM_Contact 5
enum constraint_type    {CT_TRANSLATION = 0, CT_ROTATION, CT_FULL};
enum Contact_Name       {CN_CM=0, CN_LF, CN_RF, CN_LH, CN_RH};
enum Point_Status       {PS_NO_CONTROL=0, PS_CONTACTED, PS_FLOATING};
enum leg_state          {STATE_AIR = -2, STATE_SSR, STATE_DS, STATE_SSL, STATE_QUAD};

/*!
 * \class Exception
 *
 * \brief In case of run-time error, this functrion could be called.
 *
 * It has the possibility to show an error message
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
class Exception
{
public:
    const char* msg;
    Exception(const char* arg)
    : msg(arg)
    {printf("Error: %s \n", arg);}
};

# define g 9.8
# define inf 1e20

//! Cartesian space directions, used to access vectors
enum Dir_Axis {X_DIR=0, Y_DIR, Z_DIR};

//! Thresholding a value to certain bounds
/*!
 * \param[in] in the input value
 * \param[in] up the upper bound
 * \param[in] down the lower bound
 * \param[out] double truncated
 */
double truncate_command(double in, double up, double down);

//! A zero-mean guassian random generator
/*!
 * \param[in] sigma the standard deviation
 * \param[out] double random
 */
double gauss_random(double sigma);

// some data conversion fuctions

//! Copying a matrix to a double* by scanning columns
void load_matrix(double * dest, Cmatrix in);

//! Copying a vector to a double*
void load_cvector(double * dest, Cvector in);

//! Copying a 3-element vector to a double *
void load_cvector3(double * dest, Cvector3 in);

//! Copying a double* to a 3-element vector
void save_Cvector3(Cvector3 &dest, double * in);

//! Concatenating two 3-element vectors
Cvector vector6(Cvector3 a, Cvector3 b);

//! Copying a double* to a 3-element vector
void Cvector3_convert(Cvector3& in, double out[3]);

//! Converting a double* to a 3-element vector
Cvector3 Cvector3_convert(double in[3]);

//! loading a double** to a matrix
void matrix3_convert(Cmatrix& in, double out[3][3]);

//! loading a matrix to double**
Cmatrix matrix3_convert(double in[3][3]);

//! Creating a 4x4 skew symmetric matrix
Cmatrix skew_symmetric_4d(Cvector I);

