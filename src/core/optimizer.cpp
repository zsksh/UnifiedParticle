#pragma once
#include<core/optimizer.h>

namespace UP {
	void HLBFGSOptimizer::Optimize(int N, double* init_x, int num_iter,
		int M, int T, bool with_hessian) {
		double parameter[20];
		int info[20];
		//initialize
		INIT_HLBFGS(parameter, info);
		info[3] = 0;
		info[4] = num_iter;
		info[6] = T;
		info[7] = with_hessian ? 1 : 0;
		info[10] = 0;
		info[11] = 1;

		if (with_hessian){
			HLBFGS(N, M, init_x, m_evaluation_callback, m_evalhessian_callback, HLBFGS_UPDATE_Hessian, m_newiteration_callback, parameter, info);
		}
		else{
			HLBFGS(N, M, init_x, m_evaluation_callback, 0, HLBFGS_UPDATE_Hessian, m_newiteration_callback, parameter, info);
		}
	}

	


	void HLBFGSOptimizer::optimize(double* x) {
		int T = 0;// use without hessian;
		Optimize(n_, x, max_iter_, m_, T, false);
	}

}