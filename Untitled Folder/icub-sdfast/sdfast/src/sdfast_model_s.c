/*
Generated 14-Feb-2017 19:08:47 by SD/FAST, Kane's formulation
(sdfast B.2.8 #30123) on machine ID unknown
Copyright (c) 1990-1997 Symbolic Dynamics, Inc.
Copyright (c) 1990-1997 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041
*/
#include <math.h>

/* These routines are passed to sdfast_model_root. */

void sdfast_model_posfunc(double vars[38],
    double param[1],
    double resid[38])
{
    int i;
    double pos[39],vel[38];

    for (i = 0; i < 38; i++) {
        vel[i] = 0.;
    }
    sdfast_model_ang2st(vars,pos);
    sdfast_model_state(param[0],pos,vel);
    sdfast_model_umotion(param[0],pos,vel);
    sdfast_model_perr(resid);
}

void sdfast_model_velfunc(double vars[38],
    double param[40],
    double resid[38])
{

    sdfast_model_state(param[39],param,vars);
    sdfast_model_umotion(param[39],param,vars);
    sdfast_model_verr(resid);
}

void sdfast_model_statfunc(double vars[38],
    double param[39],
    double resid[76])
{
    double pos[39],qdotdum[39];

    sdfast_model_ang2st(vars,pos);
    sdfast_model_state(param[38],pos,param);
    sdfast_model_umotion(param[38],pos,param);
    sdfast_model_uforce(param[38],pos,param);
    sdfast_model_perr(resid);
    sdfast_model_deriv(qdotdum,&resid[38]);
}

void sdfast_model_stdyfunc(double vars[76],
    double param[1],
    double resid[114])
{
    double pos[39],qdotdum[39];

    sdfast_model_ang2st(vars,pos);
    sdfast_model_state(param[0],pos,&vars[38]);
    sdfast_model_umotion(param[0],pos,&vars[38]);
    sdfast_model_uforce(param[0],pos,&vars[38]);
    sdfast_model_perr(resid);
    sdfast_model_verr(&resid[38]);
    sdfast_model_deriv(qdotdum,&resid[76]);
}

/* This routine is passed to the integrator. */

void sdfast_model_motfunc(double time,
    double state[77],
    double dstate[77],
    double param[1],
    int *status)
{
    double err[38];
    int i;

    sdfast_model_state(time,state,&state[39]);
    sdfast_model_umotion(time,state,&state[39]);
    sdfast_model_uforce(time,state,&state[39]);
    sdfast_model_deriv(dstate,&dstate[39]);
    *status = 1;
    sdfast_model_verr(err);
    for (i = 0; i < 38; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    sdfast_model_perr(err);
    for (i = 0; i < 38; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

void sdfast_model_assemble(double time,
    double state[77],
    int lock[38],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[38],param[1];
    int i;
    double jw[1444],dw[11552],rw[608];
    int iw[304],rooterr;

    sdfast_model_gentime(&i);
    if (i != 190837) {
        sdfast_model_seterr(50,42);
    }
    param[0] = time;
    sdfast_model_st2ang(state,state);
    sdfast_model_root(sdfast_model_posfunc,state,param,38,38,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,perrs,fcnt,&rooterr);
    sdfast_model_posfunc(state,param,perrs);
    *fcnt = *fcnt+1;
    sdfast_model_ang2st(state,state);
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs initial velocity analysis. */

void sdfast_model_initvel(double time,
    double state[77],
    int lock[38],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[38],param[40];
    int i;
    double jw[1444],dw[11552],rw[608];
    int iw[304],rooterr;

    sdfast_model_gentime(&i);
    if (i != 190837) {
        sdfast_model_seterr(51,42);
    }
    for (i = 0; i < 39; i++) {
        param[i] = state[i];
    }
    param[39] = time;
    sdfast_model_root(sdfast_model_velfunc,&
      state[39],param,38,38,0,lock,tol,tol,maxevals,jw,dw,rw,iw,verrs,fcnt,&
      rooterr);
    sdfast_model_velfunc(&state[39],param,verrs);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs static analysis. */

void sdfast_model_static(double time,
    double state[77],
    int lock[38],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[76],param[39],jw[2888],dw[25992],rw[874];
    int iw[456],rooterr,i;

    sdfast_model_gentime(&i);
    if (i != 190837) {
        sdfast_model_seterr(52,42);
    }
    for (i = 0; i < 38; i++) {
        param[i] = state[39+i];
    }
    param[38] = time;
    sdfast_model_st2ang(state,state);
    sdfast_model_root(sdfast_model_statfunc,state,param,76,38,38,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdfast_model_statfunc(state,param,resid);
    *fcnt = *fcnt+1;
    sdfast_model_ang2st(state,state);
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs steady motion analysis. */

void sdfast_model_steady(double time,
    double state[77],
    int lock[76],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[114],param[1],vars[76];
    double jw[8664],dw[72200],rw[1482];
    int iw[760],rooterr,i;

    sdfast_model_gentime(&i);
    if (i != 190837) {
        sdfast_model_seterr(53,42);
    }
    param[0] = time;
    sdfast_model_st2ang(state,vars);
    for (i = 0; i < 38; i++) {
        vars[38+i] = state[39+i];
    }
    sdfast_model_root(sdfast_model_stdyfunc,vars,param,114,76,38,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdfast_model_stdyfunc(vars,param,resid);
    *fcnt = *fcnt+1;
    sdfast_model_ang2st(vars,state);
    for (i = 0; i < 38; i++) {
        state[39+i] = vars[38+i];
    }
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs state integration. */

void sdfast_model_motion(double *time,
    double state[77],
    double dstate[77],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[462],ttime,param[1];
    int vintgerr,which,ferr,i;

    sdfast_model_gentime(&i);
    if (i != 190837) {
        sdfast_model_seterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        sdfast_model_motfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    sdfast_model_vinteg(sdfast_model_motfunc,&ttime,state,dstate,param,dt,&
      step,77,tol,work,&vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void sdfast_model_fmotion(double *time,
    double state[77],
    double dstate[77],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[308],ttime,param[1];
    int ferr,i;

    sdfast_model_gentime(&i);
    if (i != 190837) {
        sdfast_model_seterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        sdfast_model_motfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    sdfast_model_finteg(sdfast_model_motfunc,&ttime,state,dstate,param,dt,77,
      work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
