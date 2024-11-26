#include<fstream>
#include<filesystem>
#include<core/mesh_io.h>
namespace UP {
	void OFFMeshReader::load(const std::string& filename, Mesh& mesh) {
		std::cout << "Loading Input Surface Mesh (.OFF File) " << filename << " ..." << std::endl;
		std::ifstream fin(filename);
		if (!fin) {
			std::cerr << "can not open file " << filename << std::endl;
		}
		std::string trash;
		unsigned num_mesh_vertices, num_mesh_facets;
		double temp;
		fin >> trash >> num_mesh_vertices >> num_mesh_facets;
		mesh.clear();
		for (int n = 0; n < num_mesh_vertices; n++){
			double x, y, z;
			fin >> x >> y >> z;
			mesh.add_vertex(x, y, z);
		}
		for (int n = 0; n < num_mesh_facets; n++) {
			unsigned x, y, z;
			fin >> temp>> x >> y >> z;
			mesh.add_facet(x, y, z);
		}

	}

	void OFFMeshReader::save(const std::string& filename, Mesh& mesh) {
		std::ofstream file(filename);
		if (!file.is_open()) {
			std::cerr << "Error opening file for writing.\n";
			return;
		}

		file << "OFF\n";
		file << mesh.num_vertices() << " " << mesh.num_facets() << " 0\n";

		// Write vertices
		const std::vector<double>& vertices = mesh.get_vertices();
		for (size_t i = 0; i < vertices.size(); i += 3) {
			file << vertices[i] << " " << vertices[i + 1] << " " << vertices[i + 2] << "\n";
		}

		// Write facets (triangles)
		const std::vector<unsigned>& facets = mesh.get_facets();
		for (size_t i = 0; i < facets.size(); i += 3) {
			file << "3 " << facets[i] << " " << facets[i + 1] << " " << facets[i + 2] << "\n";
		}

		file.close();
	}



	void OBJMeshReader::load(const std::string& filename, Mesh& mesh) {
		std::cout << "Loading Input Surface Mesh (.OBJ File) " << filename << " ..." << std::endl;
		std::ifstream fin(filename);
		if (!fin) {
			std::cerr << "can not open file " << filename << std::endl;
		}
		LineInputStream in(fin);
		while (!in.eof()) {
			in.get_line();
			std::string keyword;
			in >> keyword;
			if (keyword == "v") {
				double x, y, z;
				in >> x >> y >> z;
				mesh.add_vertex(x, y, z);
			}
			else if (keyword == "f"){
				unsigned x, y, z;
				in >> x >> y >> z;
				mesh.add_facet(x, y, z);
			}
		}
	}

	void OBJMeshReader::save(const std::string& filename, Mesh& mesh) {
		std::ofstream file(filename);
		if (!file.is_open()) {
			std::cerr << "Error opening file for writing.\n";
			return;
		}

		// Write vertices
		const std::vector<double>& vertices = mesh.get_vertices();
		for (size_t i = 0; i < vertices.size(); i += 3) {
			file << "v " << vertices[i] << " " << vertices[i + 1] << " " << vertices[i + 2] << "\n";
		}

		// Write facets (faces)
		const std::vector<unsigned>& facets = mesh.get_facets();
		for (size_t i = 0; i < facets.size(); i += 3) {
			// OBJ indices are 1-based, so we add 1 to each facet index
			file << "f " << facets[i] + 1 << " " << facets[i + 1] + 1 << " " << facets[i + 2] + 1 << "\n";
		}

		file.close();

	}
	// factory method: since we need to load and save files;
	std::unique_ptr<MeshReader> create_handler(const std::string& filename) {
		std::string ext = ".obj";// std::filesystem::path(filename).extension().string();
		if (ext == ".off") return std::make_unique<OFFMeshReader>();
		else if (ext == ".obj") return std::make_unique<OBJMeshReader>();
	}

	void load_mesh(const std::string& filename, Mesh&mesh) {
		auto handler = create_handler(filename);
		handler->load(filename, mesh);
	}

	void save_mesh(const std::string& filename, Mesh& mesh) {
		auto handler = create_handler(filename);
		handler->save(filename, mesh);
	}

	void save_mesh(const std::string& filename, std::vector<double>& vertices) {
		std::ofstream file(filename);
		if (!file.is_open()) {
			std::cerr << "Error opening file for writing.\n";
			return;
		}
		for (size_t i = 0; i < vertices.size(); i += 3) {
			file << "v " << vertices[i] << " " << vertices[i + 1] << " " << vertices[i + 2] << "\n";
		}
		file.close();
	}

	void save_mesh(const std::string& filename, std::vector<float>& vertices, std::vector<int>& facets) {
		std::ofstream file(filename);
		if (!file.is_open()) {
			std::cerr << "Error opening file for writing.\n";
			return;
		}
		for (size_t i = 0; i < vertices.size(); i += 3) {
			file << "v " << vertices[i] << " " << vertices[i + 1] << " " << vertices[i + 2] << "\n";
		}

		for (size_t i = 0; i < facets.size(); i += 3) {
			// OBJ indices are 1-based, so we add 1 to each facet index
			file << "f " << facets[i] + 1 << " " << facets[i + 1] + 1 << " " << facets[i + 2] + 1 << "\n";
		}
		file.close();
	}

	void load_curvatures(const std::string& filename, std::vector<double>& max_dir,
		std::vector<double>& min_dir, std::vector<double>& normal_dir) {
		std::cout << "Loading Principal Curvatures (.CUR~ File)" << filename << std::endl;
		std::ifstream fin(filename);
		int header;
		fin >> header >> header >> header >> header >> header;

		LineInputStream in(fin);
		while (!in.eof()) {
			in.get_line();
			std::string keyword;
			in >> keyword;
			if (keyword == "v") {
				double x, y, z;
				in >> x >> y >> z;
				//mesh.add_vertex(x, y, z);
			}
			else if (keyword == "f") {
				unsigned x, y, z;
				in >> x >> y >> z;
				//mesh.add_facet(x, y, z);
			}
		}


	}




}