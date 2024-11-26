#include<iostream>
#include<algorithm>
#include<memory>
#include<numeric>
#include<cassert>
#include<random>
#include<limits>
#include<core/tangentplane.h>
#include<core/geometry.h>
#include<core/project.h>
#include<core/mesh_io.h>
#include <core/particle.h>

namespace UP {
	Particle* Particle::m_instance = nullptr;
	float M_PI = 3.1416;
	double Particle::gauss_energy(double x, double y, double z, double mu1, double mu2, double mu3, double sigma, unsigned norm) {
		double  norm_dis = pow((x - mu1), norm) + pow((y - mu2), norm) + pow((z - mu3), norm);
		norm_dis= pow(norm_dis, 2.0 / norm);
		double p1 = -norm_dis / (4 * sigma * sigma);
		double normalization = (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5)) * (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5));
		double fgauss = exp(p1) / normalization;
		return fgauss;
	}

	double Particle::gauss_grad_x(double x, double y, double z, double mu1, double mu2, double mu3,
		double sigma, unsigned norm) {
		double  dis = pow((x - mu1), norm) + pow((y - mu2), norm) + pow((z - mu3), norm);
		double norm_dis = pow(dis, 2.0 / norm);
		double p1 = -norm_dis / (4 * sigma * sigma);
		double normalization = (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5)) * (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5));
		double fgauss = exp(p1) / normalization;
		double dx= 2.0 * pow(dis, 2.0 / norm - 1.0) * pow(x - mu1, norm - 1.0)/(4 * sigma * sigma)*fgauss;
		//double dx = ((2 * (x - mu1))/(4 * sigma * sigma)) * fgauss;
		return dx;
	}

	double Particle::gauss_grad_y(double x, double y, double z, double mu1, double mu2, double mu3,
		double sigma, unsigned norm) {
		double  dis = pow((x - mu1), norm) + pow((y - mu2), norm) + pow((z - mu3), norm);
		double norm_dis = pow(dis, 2.0 / norm);
		double p1 = -norm_dis / (4 * sigma * sigma);
		double normalization = (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5)) * (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5));
		double fgauss = exp(p1) / normalization;
		double dy = 2.0 * pow(dis, 2.0 / norm - 1.0) * pow(y - mu2, norm - 1.0) / (4 * sigma * sigma) * fgauss;
		//double dy = ((2 * (y - mu2)) / (4 * sigma * sigma)) * fgauss;
		return dy;
	}
	double Particle::gauss_grad_z(double x, double y, double z, double mu1, double mu2, double mu3,
		double sigma, unsigned norm) {
			double  dis = pow((x - mu1), norm) + pow((y - mu2), norm) + pow((z - mu3), norm);
			double norm_dis = pow(dis, 2.0 / norm);
			double p1 = -norm_dis / (4 * sigma * sigma);
			double normalization = (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5)) * (sigma * sigma * sigma * pow(double(2 * M_PI), 1.5));
			double fgauss = exp(p1) / normalization;
			double dz = 2.0 * pow(dis, 2.0 / norm - 1.0) * pow(z - mu3, norm - 1.0) / (4 * sigma * sigma) * fgauss;
			//double dz = ((2 * (z - mu3)) / (4 * sigma * sigma)) * fgauss;
			return dz;
	}

	std::shared_ptr<ANNkd_tree> Particle::build_kd_tree(std::vector<double>& data, unsigned dim) {
		int size = data.size() / dim;
		ANNpointArray leaves; //data points
		leaves = annAllocPts(size, dim); // allocate data points
		for (unsigned i = 0; i < size; i++) {
			for (unsigned coord = 0; coord < dim; coord++) {
				leaves[i][coord] = data[i * dim + coord];
			}
		}
		return std::make_shared<ANNkd_tree>(leaves, size, dim);
	}
	void Particle::constrain_seeds(ANNkd_tree* tree, int k) {
		double queryPt[3] = { m_constrained_seeds[3 * k] ,m_constrained_seeds[3 * k + 1],m_constrained_seeds[3 * k + 2] };
		const int cover_num = 20;
		std::vector<unsigned> nn_idx;
		find_k_nbhd(tree, queryPt, cover_num, nn_idx);
		double distance = std::numeric_limits<double>::max();
		for (int x = 0; x < cover_num; x++) {
			auto d_idx = nn_idx[x];
			auto& n = m_discs[d_idx].m_normal;
			auto& c = m_discs[d_idx].m_center;
			auto& r = m_discs[d_idx].m_radius;
			vec3 normal = normalize(vec3(n[0], n[1], n[2]));
			double cur_dis;
			vec3 fp;
			vec3 p = vec3(m_constrained_seeds[3 * k + 0], m_constrained_seeds[3 * k + 1], m_constrained_seeds[3 * k + 2]);
			vec3 center = vec3(c[0], c[1], c[2]);
			constrained_disc_project(p, center, normal, r, fp, cur_dis);
			if (cur_dis < distance) {
				m_constrained_seeds[3 * k + 0] = fp.x;
				m_constrained_seeds[3 * k + 1] = fp.y;
				m_constrained_seeds[3 * k + 2] = fp.z;
				m_cur_normal[k] = normal;
				distance = cur_dis;
			}
		}
	}

	void Particle::cal_fun_grad(std::shared_ptr<ANNkd_tree> kd_tree, int k) {
		const int nearest_neigh = 30; 
		float radius = 5 * sqrt(double(2)) * m_sigma;
		std::vector<unsigned> res;
		find_k_nbhd_radius(kd_tree.get(), m_constrained_seeds.data()+m_dim*k, radius, nearest_neigh, res);
		for (int j = 0; j < nearest_neigh; j++) {
			int idxs = res[j];
			if (idxs > 0&& idxs!=k) {
				float temp = gauss_energy(
					m_constrained_seeds[m_dim * idxs + 0], m_constrained_seeds[m_dim * idxs + 1], m_constrained_seeds[m_dim * idxs + 2],
					m_constrained_seeds[m_dim * k + 0], m_constrained_seeds[m_dim * k + 1], m_constrained_seeds[m_dim * k + 2],
					m_sigma);

				m_grads[m_dim * k + 0] +=m_per_gaussian_energy * m_per_gaussian_energy * gauss_grad_x(
					m_constrained_seeds[m_dim * idxs + 0], m_constrained_seeds[m_dim * idxs + 1], m_constrained_seeds[m_dim * idxs + 2],
					m_constrained_seeds[m_dim * k + 0], m_constrained_seeds[m_dim * k + 1], m_constrained_seeds[m_dim * k + 2],
					m_sigma);
				m_grads[m_dim * k + 1] +=m_per_gaussian_energy * m_per_gaussian_energy * gauss_grad_y(
					m_constrained_seeds[m_dim * idxs + 0], m_constrained_seeds[m_dim * idxs + 1], m_constrained_seeds[m_dim * idxs + 2],
					m_constrained_seeds[m_dim * k + 0], m_constrained_seeds[m_dim * k + 1], m_constrained_seeds[m_dim * k + 2],
					m_sigma);
				m_grads[m_dim * k + 2] +=m_per_gaussian_energy * m_per_gaussian_energy * gauss_grad_z(
					m_constrained_seeds[m_dim * idxs + 0], m_constrained_seeds[m_dim * idxs + 1], m_constrained_seeds[m_dim * idxs + 2],
					m_constrained_seeds[m_dim * k + 0], m_constrained_seeds[m_dim * k + 1], m_constrained_seeds[m_dim * k + 2],
					m_sigma);
				m_energys[k] += m_per_gaussian_energy * m_per_gaussian_energy * temp;
			}
		}
	}
	void Particle::least_square(std::vector<double>& m_sample_para, int LBFGS_counter){
		int k = 0;
//#pragma omp parallel for
		for (k = 0; k < m_num_samples; k++) {
			constrain_seeds(m_domain_tree.get(), k);
		}
		std::shared_ptr<ANNkd_tree> kd_tree= build_kd_tree(m_sample_para, m_dim);
		m_energy = 0.0;
		m_grads.clear();
		m_grads.resize(m_num_samples * m_dim, 0);
		m_energys.clear();
		m_energys.resize(m_num_samples, 0);

		for (int k = 0; k < m_num_samples; k++) {
			cal_fun_grad(kd_tree, k);
		}
		constrain_grads();
		m_energy = std::accumulate(m_energys.begin(), m_energys.end(), float(0.0));
	}

	void Particle::constrain_grads() {
		for (int k = 0; k < m_num_samples; k++) {
			vec3 grad = vec3(m_grads[m_dim * k + 0], m_grads[m_dim * k + 1], m_grads[m_dim * k + 2]);
			vec3 projected_grad = grad - m_cur_normal[k] * dot(grad, m_cur_normal[k]);
			for (unsigned coord = 0; coord < m_dim; coord++) {
				m_grads[m_dim * k + coord] = projected_grad[coord];
			}
		}
	}

	void Particle::optimization_bfgs(unsigned int nb_iter, unsigned int m, int size,
		std::vector<double>& points, typename HLBFGSOptimizer::evaluation_callback fp){
		auto optimizer = std::make_unique<HLBFGSOptimizer>();
		unsigned n = unsigned(points.size());
		optimizer->set_epsg(0.0);
		optimizer->set_epsf(0.0);
		optimizer->set_epsx(0.0);
		optimizer->set_newiteration_callback(newiteration_CB);
		optimizer->set_evaluation_callback(fp);
		optimizer->set_N(n);
		optimizer->set_M(m);
		optimizer->set_max_iter(nb_iter);
		optimizer->optimize(points.data());
	}

	void Particle::remesh(int num_itr) {
		std::cout << "L-BFGS Optimizing.." << std::endl;
		optimization_bfgs(num_itr, 7, m_seeds.size(), m_seeds, evaluation_callback);
	}


	void Particle::cal_fun_grad(double* x, double* f, double* g) {
		*f = 0;
		std::copy(x, x + m_dim * m_num_samples, m_constrained_seeds.begin());
		least_square(m_constrained_seeds, m_LBFGS_iter);
		std::copy(m_grads.begin(), m_grads.end(), g);
		if (m_LBFGS_iter == 0) {
			*f = m_energy;
			m_pre_energy = *f;
		}
		if (m_energy <= m_pre_energy){
			m_iter_counter++;
			m_LBFGS_iter++;
			m_pre_energy = m_energy;
			std::cout << "Iteration: " << m_LBFGS_iter <<" f-value: " << m_energy << std::endl;
		}
		*f = m_energy;
	}

	Particle::Particle(std::shared_ptr<Mesh> mesh, unsigned int  num_sample) :m_num_samples(num_sample) {
		m_iter_counter = 0;
		m_LBFGS_iter = 0;
		m_domain_tree = nullptr;
		m_cur_normal.resize(num_sample);
		m_constrained_seeds.resize(m_dim * num_sample);
		m_mesh = mesh;
		m_vertices = m_mesh->get_vertices();
		m_num_mesh_vertices = m_vertices.size()/m_dim;
		m_instance = this;
	}
	void Particle::initial_sampling(){
		assert(m_num_samples < m_num_mesh_vertices);
		std::cout << "Initializing Sampling Points..." << std::endl;
		std::cout << "Number of  Sampling Points: " << m_num_samples << std::endl;
		if (!m_mesh) {
			std::cerr << "Mesh not initialized!" << std::endl;
			return;
		}
		m_seeds.resize(m_num_samples *m_dim, 0);
		const int total_vertices = m_mesh->num_vertices();
		assert(total_vertices > m_num_samples);
		std::vector<int> vertex_indices(total_vertices);
		for (int i = 0; i < total_vertices; ++i) {
			vertex_indices[i] = i;
		}
		std::shuffle(vertex_indices.begin(), vertex_indices.end(), std::default_random_engine(0));
		for (int i = 0; i < m_num_samples; i++) {
			auto idx = vertex_indices[i];
			for (unsigned coord = 0; coord < m_dim; coord++) {
				m_seeds[m_dim * i + coord] = m_vertices[m_dim * idx + coord];
			}
		}
	}

	void Particle::build_kd_tree() {
		m_domain_tree = build_kd_tree(m_vertices, m_dim);
	}

	void Particle::ouput_underlying_surface(std::string filename) {
		auto num_disc_edge = 10;
		std::vector <float> vertices;
		std::vector<int> facets;
		for (int k = 0; k < m_discs.size(); k++) {
			int starting_idx = vertices.size() / m_dim;
			auto& c = m_discs[k].m_center;
			vec3 center= vec3(c[0], c[1], c[2]);
			auto& n = m_discs[k].m_normal;
			vec3 normal = vec3(n[0], n[1],n[2]); // get normal
			auto& r = m_discs[k].m_radius;
			std::vector<vec3> res;
			generate_disc(center,res, normal, m_discs[k].m_radius);
			for (auto p : res) {
				vertices.push_back(p[0]);
				vertices.push_back(p[1]);
				vertices.push_back(p[2]);
			}
			for (int t = 0; t < num_disc_edge-1; t++){
				facets.push_back(starting_idx);
				facets.push_back(starting_idx+(t+1));
				facets.push_back(starting_idx+(t+2));
			}
		}
		save_mesh(filename, vertices, facets);

	}

	void Particle::calculate_sigma() {
		std::cout << "Calculating Gaussian Sigma and Surface Areas in Original Space..." << std::endl;
		double area_surface = 0;
		//#pragma omp parallel for
		for (int i = 0; i < m_num_mesh_vertices; i++) {
			double edge_length = 0;
			std::vector<unsigned> res;
			find_k_nbhd(m_vertices.data() + m_dim * i, 30, res);
			for (int m = 0; m < 6; m++) {
				edge_length += distance_points_3D(m_vertices[3 * i + 0], m_vertices[3 * i + 1], m_vertices[3 *i + 2],
					m_vertices[3 * res[m] + 0], m_vertices[3 * res[m] + 1], m_vertices[3 * res[m] + 2]);
			}
			area_surface += 0.86 /25 * edge_length * edge_length ;
		}
		m_sigma = 0.32 * sqrt(double(area_surface / m_num_samples));
		m_per_gaussian_energy = area_surface / m_num_samples; //  
		std::cout << "sigma:=" << m_sigma << std::endl;

	}

	void  Particle::find_k_nbhd(double* query_pts, unsigned nb, std::vector<unsigned>& res) {
		std::vector<int> nn_idx(nb);
		std::vector<double> dists(nb);
		m_domain_tree->annkSearch(query_pts, nb, nn_idx.data(), dists.data(), 0.0);
		for (auto i : nn_idx) {
			res.push_back(i);
		}
	}

	void  Particle::find_k_nbhd(ANNkd_tree* kd_tree, double* query_pts, unsigned nb, std::vector<unsigned>& res) {
		std::vector<int> nn_idx(nb);
		std::vector<double> dists(nb);
		kd_tree->annkSearch(query_pts, nb, nn_idx.data(), dists.data(), 0.0);
		for (auto i : nn_idx) {
			res.push_back(i);
		}
	}

	void  Particle::find_k_nbhd_radius(double* query_pts, double radius,unsigned nb, std::vector<unsigned>& res) {
		std::vector<int> nn_idx(nb);
		std::vector<double> dists(nb);
		m_domain_tree->annkFRSearch(query_pts, radius * radius, nb, nn_idx.data(), dists.data(), 0.0);
		for (auto i : nn_idx) {
			res.push_back(i);
		}
	}
	void  Particle::find_k_nbhd_radius(ANNkd_tree* kd_tree, double* query_pts,double radius, unsigned nb, std::vector<unsigned>& res) {
		std::vector<int> nn_idx(nb);
		std::vector<double> dists(nb);
		kd_tree->annkFRSearch(query_pts, radius * radius, nb, nn_idx.data(), dists.data(), 0.0);
		for (auto i : nn_idx) {
			res.push_back(i);
		}
	}


	void Particle::final_sampling(std::string filename){
		save_mesh(filename, m_constrained_seeds);
	}

	void Particle::approximate_underlying_surface() {
		std::cout << "pre-calculate norms and centers" << std::endl;
		m_discs.resize(m_num_mesh_vertices);
#pragma omp parallel for
		for (int i = 0; i < m_num_mesh_vertices; i++) {
			std::vector<double> pcl;
			std::vector<unsigned> res;
			find_k_nbhd(m_vertices.data()+m_dim*i, 30, res);
			m_discs[i].m_radius = 0.75 * distance_points_3D(m_vertices[m_dim * i + 0], m_vertices[m_dim * i + 1], m_vertices[m_dim * i + 2],
				m_vertices[m_dim * res[5] + 0], m_vertices[m_dim * res[5] + 1], m_vertices[m_dim * res[5] + 2]);
			for (int i = 0; i < 30; i++) {
				pcl.push_back(m_vertices[m_dim * res[i] + 0]);
				pcl.push_back(m_vertices[m_dim * res[i] + 1]);
				pcl.push_back(m_vertices[m_dim * res[i] + 2]);
			}
			Tangentplane plane;
			plane.compute(pcl);
			m_discs[i].set_from(plane, m_vertices.data()+m_dim*i);
		}
		//ouput_underlying_surface("output/underlying_surface.obj");
		std::cout << "Done." << std::endl;
	}
}