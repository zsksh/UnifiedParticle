#pragma once
#include<vector>
#include<core/plane.h>
#include<Eigen/Dense>
#include<core/vecg.h>
namespace UP {


	/*template <class FT, unsigned M_DIM>
	inline typename std::enable_if<M_DIM != 3>::type
		compute_tangent_plane(std::vector<FT>& pcl) {
		std::cout << "General implementation for M_DIM != 3." << std::endl;
	}

	// Specialized implementation for M_DIM == 3
	template <class FT, unsigned M_DIM>
	inline typename std::enable_if<M_DIM == 3>::type
		compute_tangent_plane(std::vector<FT>& pcl) {
		if (pcl.empty()) {
			throw std::invalid_argument("Point cloud is empty.");
		}

		// Step 1: Compute the centroid of the point cloud
		Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
		for (const auto& point : pcl) {
			centroid += point;
		}
		centroid /= pcl.size();

		// Step 2: Construct the covariance matrix
		Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
		for (const auto& point : pcl) {
			Eigen::Vector3d centered = point - centroid;
			covariance += centered * centered.transpose();
		}
		covariance /= pcl.size();

		// Step 3: Perform eigen decomposition of the covariance matrix
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
		if (solver.info() != Eigen::Success) {
			throw std::runtime_error("Eigen decomposition failed.");
		}

		// Step 4: Extract the normal of the tangent plane
		// The eigenvector corresponding to the smallest eigenvalue is the normal
		int minIndex;
		eigenValues.minCoeff(&minIndex); // Find the index of the smallest eigenvalue
		tangentPlaneNormal = solver.eigenvectors().col(minIndex);
		planePoint = centroid;

	}
	*/


	class Tangentplane :public Plane<double> {
		using vec3 = vec3g<double>;


	private:
		Eigen::MatrixXd data;
		Eigen::VectorXd eigenValues;
		Eigen::MatrixXd eigenVectors;

		Eigen::VectorXd convertedOrigin;


	public:
		double v[3];
		void PCA(const Eigen::MatrixXd& data, bool normalizeEachData = true, bool normalizeOrigin = false);
		//void compute(std::vector<double>& knnx, std::vector<double>& knny, std::vector<double>& knnz);
		void compute(std::vector<double>& pcl);

		Eigen::MatrixXd getEigenVectors() const
		{
			return eigenVectors;
		}
		//double distance(Point& point)const { return abs(a*point.x + b*point.y + c*point.z + d) / sqrt(a*a + b*b + c*c); }
		double distance(vec3& point)const { 
			return abs(m_a * point.x + m_b * point.y + m_c * point.z + m_d)
				/ sqrt(m_a * m_a + m_b * m_b + m_c * m_c); 
		}
		Eigen::VectorXd getEigenValues() const
		{
			return eigenValues;
		}

		Eigen::VectorXd getConvertedOrigin() const
		{
			return convertedOrigin;
		}

	};

}

