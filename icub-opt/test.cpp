//#include "basics.h"
#include "iCubOpt.hpp"
#include "Model.hpp"

using namespace std;

#define Cold 0
#define Basis 1
#define Warm 2
#define num_run 1e2


int main( int argc, char **argv)
{
	iCubOpt iCub_Opt;
	iCub_Opt.init();

	int num_iter = 1000;
	double elastic = 1e15;

	iCub_Opt.W_STATIC_init();
	vars *v = &iCub_Opt.W_STATIC_vars;

	int flag = 0;
	iCub_Opt.counter = 0 ;

	snopt_set_parameters(*v, num_iter, 6, 0, 3, 1e-12	, 1e-6);
	//      v->snopt->setRealParameter ( "elastic weight", pow(2.0,k)*0+elastic);

	flag = v->snopt->solve(Cold);
	//iCub_Opt.W_STATIC_shot(v->snopt->x,v->F,v->G);
	std::cout << "************************************  AFTER FIRST SOLVE "  <<std::endl;



	ofstream icub_opt ;
	icub_opt.open( "iCub_Opt.txt", std::ofstream::out | std::ofstream::app );

	ofstream const_limit ;
	const_limit.open( "const_limit.txt", std::ofstream::out | std::ofstream::app );

	for (int i = 0; i < v->num_eq; i++){

		if (v->F[i] > v->Fupp[i] ||  v->F[i] < v->Flow[i])
		{
			const_limit << "Step No. = "<< iCub_Opt.counter << "-  F(" << i << ")= "<< v->F[i] << "  Fupper = " << v->Fupp[i] <<  "  Flow = " << v->Flow[i] << endl;

		}

	}

	const_limit << "========================================Next Step ======="  <<  endl;


	ofstream acc_data ;
	acc_data.open( "acc_data.txt", std::ofstream::out | std::ofstream::app );
	acc_data <<  v->x[35] << endl;
	//acc_data <<  v->G << std::endl;

	ofstream dir_data ;
	dir_data.open( "dir_data.txt", std::ofstream::out | std::ofstream::app );
	dir_data <<  iCub_Opt.counter  << " - F(0) = " << v->F[0] << "  Fupper = " << v->Fupp[0] <<  "  Flow = " << v->Flow[0] << endl;
	 dir_data <<  iCub_Opt.counter << " - F(1) = " << v->F[1] << "  Fupper = " << v->Fupp[1] <<  "  Flow = " << v->Flow[1] << endl;


	for (int i = 0; i < v->num_var; i++){
		icub_opt << "x = " << v-> x[i] <<   "  Xupper = " << v->xupp[i] <<  "  Xlow = " << v->xlow[i] << endl;

	}
	for (int i = 0; i < v->num_var; i++){
			icub_opt << "x0 = " << v-> x0[i] << endl;

		}
	for (int i = 0; i < v->num_eq; i++){
		icub_opt << "F = " << v->F[i] << "  Fupper = " << v->Fupp[i] <<  "  Flow = " << v->Flow[i] << endl;

	}

	icub_opt <<"===============================================Next Step " << iCub_Opt.counter   <<std::endl;


	iCub_Opt.update(v->x);

	iCub_Opt.counter = iCub_Opt.counter + 1 ;

	iCub_Opt.epsilon = 1;
	while (iCub_Opt.epsilon > 1e-02 || iCub_Opt.epsilon < -1e-02)
	{


		iCub_Opt.W_STATIC_update(v->x);

		vars *v = &iCub_Opt.W_STATIC_vars;

		int flag = 0;

		snopt_set_parameters(*v, num_iter, 6, 0, 3, 1e-12	, 1e-6);
		//      v->snopt->setRealParameter ( "elastic weight", pow(2.0,k)*0+elastic);

		flag = v->snopt->solve(Cold);


		for (int i = 0; i < v->num_eq; i++){

			if (v->F[i] > v->Fupp[i] ||  v->F[i] < v->Flow[i])
			{
				const_limit << "Step No. = "<< iCub_Opt.counter << "-  F(" << i << ")= "<< v->F[i] << "  Fupper = " << v->Fupp[i] <<  "  Flow = " << v->Flow[i] << endl;

			}

		}

		const_limit << "========================================Next Step ======="  <<  endl;



		dir_data <<  iCub_Opt.counter  << " - F(0) = " << v->F[0] << "  Fupper = " << v->Fupp[0] <<  "  Flow = " << v->Flow[0] << endl;
		// dir_data <<  iCub_Opt.counter << " - F(77) = " << v->F[77] << "  Fupper = " << v->Fupp[77] <<  "  Flow = " << v->Flow[77] << endl;


		for (int i = 0; i < v->num_var; i++){
			icub_opt << "x = " << v-> x[i] <<   "  Xupper = " << v->xupp[i] <<  "  Xlow = " << v->xlow[i] << endl;

		}
		for (int i = 0; i < v->num_var; i++){
					icub_opt << "x0 = " << v-> x0[i] << endl;

				}
		for (int i = 0; i < v->num_eq; i++){
			icub_opt << "F = " << v->F[i] << "  Fupper = " << v->Fupp[i] <<  "  Flow = " << v->Flow[i] << endl;

		}
		icub_opt <<"===============================================Next Step " << iCub_Opt.counter   <<std::endl;
		//  icub_opt << iCub_Opt.counter <<"th step is done. " <<std::endl;

		// cout << iCub_Opt.counter <<"th step is done. " <<std::endl;
		acc_data <<  v->x[35] << std::endl;


		iCub_Opt.update(v->x);

		//iCub_Opt.W_STATIC_delete();
		iCub_Opt.counter = iCub_Opt.counter + 1 ;
		std::cout << "***************************************Next Iteration "  <<  iCub_Opt.counter << std::endl;

	}



	return 0;
}

