#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "snoptProblem.hpp"

struct vars
{
    int     num_var;
    int     num_eq;
    int     num_deriv;
    int     num_hessian;
    int     num_const;
    int     ObjRow;
    double  ObjAdd;

    int*    iAfun;
    int*    jAvar;
    double* A;
    int*    iGfun;
    int*    jGvar;
    double* G;
    int*    iHfun;
    int*    jHvar;
    double* H;

    double* x0;
    double* x;
    double* xlow;
    double* xupp;
    double* xmul;
    int*    xstate;

    double* F;
    double* Flow;
    double* Fupp;
    double* Fmul;
    int*    Fstate;

    snoptProblemA *snopt;
};

inline void snopt_allocate_space(vars &v)
{
    v.iAfun = new int   [v.num_var*v.num_eq];
    v.jAvar = new int   [v.num_var*v.num_eq];
    v.A     = new double[v.num_var*v.num_eq];
    v.iGfun = new int   [v.num_var*v.num_eq];
    v.jGvar = new int   [v.num_var*v.num_eq];
    v.G     = new double[v.num_var*v.num_eq];
    v.iHfun = new int   [v.num_var*v.num_var];
    v.jHvar = new int   [v.num_var*v.num_var];
    v.H     = new double[v.num_var*v.num_var];

    v.x0     = new double[v.num_var];
    v.x      = new double[v.num_var];
    v.xlow   = new double[v.num_var];
    v.xupp   = new double[v.num_var];
    v.xmul   = new double[v.num_var];
    v.xstate = new    int[v.num_var];

    v.F      = new double[v.num_eq];
    v.Flow   = new double[v.num_eq];
    v.Fupp   = new double[v.num_eq];
    v.Fmul   = new double[v.num_eq];
    v.Fstate = new    int[v.num_eq];
}

inline void snopt_delete_space(vars &v)
{
    delete []v.iAfun;  delete []v.jAvar;  delete []v.A;
	delete []v.iGfun;  delete []v.jGvar;  delete []v.G;
	delete []v.iHfun;  delete []v.jHvar;  delete []v.H;

	delete []v.x;      delete []v.xlow;   delete []v.xupp;
	delete []v.xmul;   delete []v.xstate; delete []v.x0;

	delete []v.F;      delete []v.Flow;   delete []v.Fupp;
	delete []v.Fmul;   delete []v.Fstate;
}

inline void snopt_set_parameters(vars &v,
                                int MajorIterationLimit=100,
                                int SummaryFile=0,
                              //  int PrintFile = 1,
                             //   int ElasticWeight=0.0,
                                int DerivativeOption=0, // 0 means some derivatives are unknown
                                int VerifyLevel=3,
                           //     double LineSearchTolerance=0.01 ,
                                //double ViolationLimit = 0.0,
                                double MajorOptimalityTolerance=1e-2,
                                double MajorFeasibilityTolerance=1e-6)
{
    v.snopt->setIntParameter( "Summary file", SummaryFile);
   // v.snopt->setIntParameter( "Elastic weight", ViolationLimit);
 //   v.snopt->setIntParameter( "Violation limit", ElasticWeight);
  //  v.snopt->setIntParameter( "Print file", PrintFile);
    v.snopt->setIntParameter( "Derivative option", DerivativeOption);  //
    v.snopt->setIntParameter( "verify level", VerifyLevel);
   // v.snopt->setIntParameter( "Line Search Tolerance", LineSearchTolerance);
    v.snopt->setIntParameter( "major iterations limit", MajorIterationLimit);
    v.snopt->setRealParameter( "major optimality tolerance", MajorOptimalityTolerance);
    v.snopt->setRealParameter( "major feasibility tolerance", MajorFeasibilityTolerance);
}

// for the case with derivatives
inline void snopt_load_vars_full(vars &v)
{
    memcpy(v.x, v.x0, v.num_var*sizeof(double));
    v.snopt->setProblemSize( v.num_var, v.num_eq );
    v.snopt->setObjective  ( v.ObjRow, v.ObjAdd );
    v.snopt->setX          ( v.x, v.xlow, v.xupp, v.xmul, v.xstate );
    v.snopt->setF          ( v.F, v.Flow, v.Fupp, v.Fmul, v.Fstate );
    v.snopt->setA          ( v.num_var*v.num_eq, v.iAfun, v.jAvar, v.A );
    v.snopt->setG          ( v.num_var*v.num_eq, v.iGfun, v.jGvar );
    v.snopt->setNeA        ( v.num_const*0 );
    v.snopt->setNeG        ( v.num_deriv);
}

inline void print_vector(const double x[], int length, const char dest[], const char filename[], int num=0)
{
    std::ofstream outX;
    char str[80];
    sprintf(str,"%s%s%d.txt", dest, filename, num);
    outX.open(str);
    for(int i=0;i<length;i++)
        outX << std::setprecision(12) << x[i] << std::endl;
    outX.close();
}

inline void load_vector(double x[], int length, const char dest[], const char filename[], int num=0)
{
    std::ifstream inX;
    char str[80];
    sprintf(str,"%s%s%d.txt", dest, filename, num);
    inX.open(str);
    for(int i=0;i<length;i++)
        inX >> x[i];
    inX.close();
}



