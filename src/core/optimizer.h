#pragma once
#include<functional>
#include <HLBFGS/HLBFGS.h>
#include <HLBFGS/Lite_Sparse_Matrix.h>
namespace UP {

	class HLBFGSOptimizer {
		/*using newiteration_callback = std::function<void(
			int N, double* x, double f, double* g, double gnorm
			)>;
		using evalhessian_callback = std::function<void(
			int N, double* x, double& f, double* g, HESSIAN_MATRIX& hessian)>;
		 
		using funcgrad_callback = std::function<void(
			unsigned N, double* x, double& f, double* g
			)>;

		typedef void (*funcgrad_callback)(
			index_t N, double* x, double& f, double* g
			);*/

	public:
		typedef void (*funcgrad_callback)(
			int N, double* x, double& f, double* g
			);

		typedef void (*newiteration_callback)(
			int,int, double*, double*, double*, double*
			);
		typedef void (*evaluation_callback)(int, double*, double*,
			double*, double*);
		typedef void (*evalhessian_callback)(
			int N, double* x, double*,double* f, double* g, HESSIAN_MATRIX& hessian
			);
	public:
		void Optimize(int N, double* init_x, int num_iter, int M, int T, bool with_hessian);
		void set_newiteration_callback(newiteration_callback fp) { m_newiteration_callback = fp; }
		void set_evaluation_callback(evaluation_callback fp) {
			m_evaluation_callback = fp;
		}
		void set_epsg(double eg) {epsg_ = eg;}
		void set_epsf(double ef) {epsf_ = ef;}
		void set_epsx(double ex) {epsx_ = ex;}

		void set_N(int N) {n_ = N;}
		int get_N() const {return n_;}
		void set_M(int M) {m_ = M;}
		int get_M() const {return m_;}

		void set_max_iter(int maxiter) {max_iter_ = maxiter;}

		int get_max_iter() const {return max_iter_;}

		void optimize(double* x);

	private:

		newiteration_callback m_newiteration_callback;
		funcgrad_callback m_funcgrad_callback;

		evalhessian_callback m_evalhessian_callback;
		evaluation_callback  m_evaluation_callback;


		/** Error tolerance on x, f and g */
		double epsg_, epsf_, epsx_;
		int n_, m_;

		int max_iter_;
	};


}