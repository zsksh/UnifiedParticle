#include <iostream>
#include<core/mesh_io.h>
#include<core/mesh.h>
#include<core/particle.h>
using namespace std;
using namespace UP;
//extern  Particle* theKNL;
void main(int argc, char ** argv){
	int num_sample =2000;
	int num_itr = 10000;
	std::string filename = "kitten1280k.obj";
	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	load_mesh(filename, *(mesh.get()));

	auto particle = new Particle(mesh,num_sample);
	particle->initial_sampling();
	particle->build_kd_tree();
	particle->approximate_underlying_surface();
	particle->calculate_sigma();
	particle->remesh(10000);
	particle->final_sampling("output/kitten1280k.obj");


}