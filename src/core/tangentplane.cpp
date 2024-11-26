#include"tangentplane.h"
#include<string>
#include<fstream>

namespace UP {
	void Tangentplane::PCA(const Eigen::MatrixXd& data, bool normalizeEachData, bool normalizeOrigin)
		//:data(data), convertedOrigin(Eigen::VectorXd::Zero(data.cols()))
	{
		//std::cout << "data" << std::endl << data << std::endl;
		const int DIM = (int)data.cols();
		Eigen::MatrixXd W = data;//size=0.0
		Eigen::MatrixXd covarianceMatrix = data; // size=0.0

		if (normalizeEachData) {
			Eigen::VectorXd mean = data.colwise().mean();
			//std::cout << mean << std::endl;
			W = (W.rowwise() - mean.transpose());

			//std::cout<<W<<std::endl;
		}
		if (normalizeOrigin) {
			convertedOrigin = data.colwise().mean();
			W = W.rowwise() - convertedOrigin.transpose();
		}
		covarianceMatrix = W.transpose() * W;

		//covarianceMatrix.normalize();
	//		std::cout << "covarianceMatrix" << std::endl << covarianceMatrix << std::endl;

			//calculate eigens
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
		Eigen::VectorXd eigenValues = solver.eigenvalues();
		Eigen::MatrixXd eigenVectors = solver.eigenvectors();

		//	std::cout << "eigenvalues: "<<std::endl << eigenValues << std::endl;
		//	std::cout << "eigenvectors: " <<std::endl<< eigenVectors << std::endl;

			//make eigenvalues and eigen matrix
		this->eigenValues = Eigen::VectorXd(DIM);
		this->eigenVectors = Eigen::MatrixXd(DIM, DIM);
		for (int i = 0; i < DIM; i++) {
			int r = DIM - i - 1;
			this->eigenValues(i) = eigenValues(r);
			this->eigenVectors.col(i) = eigenVectors.col(r);
		}
	}
	void Tangentplane::compute(std::vector<double>& pcl) {
		int size = pcl.size()/3;
		data = Eigen::MatrixXd(size, 3);// size*3
		for (int i = 0; i < size; i++) {
			data(i, 0) = pcl[3 * i + 0];
			data(i, 1) = pcl[3 * i + 1];
			data(i, 2) = pcl[3 * i + 2];
		}

		//std::cout << data << std::endl;
		convertedOrigin = Eigen::VectorXd::Zero(data.cols());
		PCA(data);
		int minIndex;
		eigenValues.minCoeff(&minIndex); // Find the index of the smallest eigenvalue
		m_a = this->eigenVectors.col(minIndex)[0];
		m_b = this->eigenVectors.col(minIndex)[1];
		m_c = this->eigenVectors.col(minIndex)[2];
	}



}



