#pragma once
#include<ANN/ANN.h>
#include<core/plane.h>
#include<core/mesh.h>
#include<core/vecg.h>
#include<core/disc.h>
#include<core/optimizer.h>
#include<string>
#include<vector>
#include<array>
namespace UP {
	class Particle {
		unsigned const m_dim = 3;
		using Disc3 = Disc<double>;
		using vec3 = vec3g<double>;
	public:
		Particle(std::shared_ptr<Mesh> mesh, unsigned int  num_sample);
		void  initial_sampling();
		void  approximate_underlying_surface();
		void  calculate_sigma();
		void   build_kd_tree();
		void   remesh(int num_itr);
		void   final_sampling(std::string filename);
		~Particle() = default;

	private:
		double gauss_energy(double x, double y, double z, double mu1, double mu2, double mu3, double sigma, unsigned norm =8);
		double gauss_grad_x(double x, double y, double z, double mu1, double mu2, double mu3, double sigma, unsigned norm = 8);
		double gauss_grad_y(double x, double y, double z, double mu1, double mu2, double mu3, double sigma, unsigned norm = 8);
		double gauss_grad_z(double x, double y, double z, double mu1, double mu2, double mu3, double sigma, unsigned norm = 8);


		//float	gauss_energy(float x, float y, float z, float mu1, float mu2, float mu3, float sigma, float M11, float M12, float M13, float M21, float M22, float M23, float M31, float M32, float M33);
		//float	gauss_grad_x(float x, float y, float z, float mu1, float mu2, float mu3, float sigma, float M11, float M12, float M13, float M21, float M22, float M23, float M31, float M32, float M33, float Q11, float Q12, float Q13, float Q21, float Q22, float Q23, float Q31, float Q32, float Q33);
		//float	gauss_grad_y(float x, float y, float z, float mu1, float mu2, float mu3, float sigma, float M11, float M12, float M13, float M21, float M22, float M23, float M31, float M32, float M33, float Q11, float Q12, float Q13, float Q21, float Q22, float Q23, float Q31, float Q32, float Q33);
		//float	gauss_grad_z(float x, float y, float z, float mu1, float mu2, float mu3, float sigma, float M11, float M12, float M13, float M21, float M22, float M23, float M31, float M32, float M33, float Q11, float Q12, float Q13, float Q21, float Q22, float Q23, float Q31, float Q32, float Q33);


		void  least_square(std::vector<double>& sample_para, int LBFGS_counter);
		void  cal_fun_grad(double* x, double* f, double* g);
		void  constrain_grads();
		void  constrain_seeds(ANNkd_tree* tree, int index);
		void  ouput_underlying_surface(std::string filename);
		void  find_k_nbhd(double* query_pts, unsigned nb, std::vector<unsigned>& res);
		void  find_k_nbhd(ANNkd_tree* kd_tree, double* query_pts, unsigned nb, std::vector<unsigned>& res);
		void  find_k_nbhd_radius(double* query_pts, double radius, unsigned nb, std::vector<unsigned>& res);
		void  find_k_nbhd_radius(ANNkd_tree* kd_tree, double* query_pts, double radius, unsigned nb, std::vector<unsigned>& res);
		std::shared_ptr<ANNkd_tree> build_kd_tree(std::vector<double>& data, unsigned dim = 3);
	private:
		int m_num_samples;
		int	m_num_mesh_vertices;
		float  m_sigma;
		std::shared_ptr<Mesh> m_mesh;
		std::shared_ptr<ANNkd_tree> m_domain_tree;
		std::vector<vec3>  m_cur_normal;
		std::vector<double> m_constrained_seeds;
		std::vector <double> m_seeds;
		std::vector<Disc3> m_discs;

	private:
		float  m_energy;
		float  m_pre_energy;
		float  m_per_gaussian_energy;
		int  m_iter_counter;
		int  m_LBFGS_iter;
		std::vector<float> m_grads;
		std::vector<float> m_energys;
		std::vector <double> m_vertices;
	public:
		static void newiteration_CB(int n, int m, double* x, double* f,
			double* g, double* gnorm) {
			m_instance->newiteration(n, m, x, f, g, gnorm);
		}
		void newiteration(int n, int m, double* x, double* f,
			double* g, double* gnorm) {
			//std::cout << "f: " << *f << " " << "g: " << *gnorm << std::endl;
		}
		static void evaluation_callback(int n, double* x, double* prev_x, double* f, double* g) {
			m_instance->cal_fun_grad(x, f, g);
		};

		void optimization_bfgs(unsigned int nb_iter, unsigned int m, int size,
			std::vector<double>& points, typename HLBFGSOptimizer::evaluation_callback fp);
		void cal_fun_grad(std::shared_ptr<ANNkd_tree> kd_tree, int k);
	public:
		static Particle* m_instance;
	};
};




