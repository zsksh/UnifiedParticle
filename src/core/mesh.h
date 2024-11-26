#pragma once
#include<vector>
namespace UP {
	class Mesh {
	public:
		Mesh() = default;
		explicit Mesh(const std::vector<double>& vertices) { m_vertices = vertices; }
		void clear() { m_vertices.clear(), m_facets.clear();}
		unsigned num_vertices() const { return m_vertices.size() / 3; }
		unsigned num_facets() const { return m_facets.size() / 3; }
		const std::vector<double>& get_vertices() const {
			return m_vertices;
		}

		const std::vector<unsigned>& get_facets() const {
			return m_facets;
		}
		void add_vertex(double x, double y, double z) {
			m_vertices.push_back(x);
			m_vertices.push_back(y); 
			m_vertices.push_back(z);
		}
		void add_facet(unsigned x, unsigned y, unsigned z) {
			m_facets.push_back(x);
			m_facets.push_back(y);
			m_facets.push_back(z);
		}
	private:
		std::vector<double> m_vertices;
		std::vector<unsigned> m_facets;

	};


	/*class MeshDisc {
	public:
		explicit MeshDisc(std::shared_ptr<Mesh> mesh) : m_mesh{ mesh } {
			auto num_vertices = m_mesh->num_vertices();
			m_normals.resize(num_vertices * 3, 0.0f);
			m_centers = m_mesh->get_vertices();
			m_radius.resize(num_vertices * 3, 0);
		};


	private:
		std::vector<double> m_normals;
		std::vector<double> m_centers;
		std::vector<double> m_radius;
		std::shared_ptr<Mesh> m_mesh;
	};*/

}