#pragma once
namespace UP {

	const double sincos_table[10][2] = {
		{ 0, 1 },
		{ 0.642788, 0.766044 },
		{ 0.984808, 0.173648 },
		{ 0.866025, -0.5 }, // 60
		{ 0.34202, -0.939693 },
		{ -0.34202, -0.939693 },
		{ -0.866025, -0.5 },
		{ -0.984808, 0.173648 },
		{ -0.642788, 0.766044 },
		{ -2.44929e-16, 1 }
	};
	template <typename FT, typename UT>
	inline FT distance(FT* p, FT* q, UT dim) {
		FT res = 0;
		for (unsigned coord = 0; coord < dim; coord++) {
			res += (p[coord] - q[coord]) * (p[coord] - q[coord]);
		}
		return sqrt(res);
	}

	template <typename FT>
	FT distance_points_3D(FT x1, FT y1, FT z1, FT x2, FT y2, FT z2) {
		FT dis = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
		return dis;
	}
	template <typename FT>
	FT  Triangle_area(FT vertex1_x, FT vertex1_y, FT vertex1_z, FT vertex2_x,
		FT vertex2_y, FT vertex2_z, FT vertex3_x, FT vertex3_y, FT vertex3_z) {
		FT tri_area;
		FT a = distance(vertex1_x, vertex1_y, vertex1_z, vertex2_x, vertex2_y, vertex2_z);
		FT b = distance(vertex1_x, vertex1_y, vertex1_z, vertex3_x, vertex3_y, vertex3_z);
		FT c = distance(vertex2_x, vertex2_y, vertex2_z, vertex3_x, vertex3_y, vertex3_z);
		tri_area = 0.25 * sqrt(FT((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c)));
		return tri_area;
	}


	template <class vec3>
	vec3 perpendicular(const vec3& V) {
		int min_index = 0;
		double c = ::fabs(V[0]);
		double cur = ::fabs(V[1]);
		if (cur < c) {
			min_index = 1;
			c = cur;
		}
		cur = ::fabs(V[2]);
		if (cur < c) {
			min_index = 2;
		}
		vec3 result;
		switch (min_index) {
		case 0:
			result = vec3(0, -V.z, V.y);
			break;
		case 1:
			result = vec3(V.z, 0, -V.x);
			break;
		case 2:
			result = vec3(-V.y, V.x, 0);
			break;
		}
		return result;
	}

	template <class Vec3, class T>
	void generate_disc(const Vec3& center, std::vector<Vec3>& res, const Vec3& normal, T radius, unsigned nb = 10) {
		res.clear();
		Vec3 u = perpendicular(normal);
		u = normalize(u);
		Vec3 v = cross(normal, u);
		v = normalize(v);
		res.push_back(center);// all need centers;
		for (int k = 0; k < nb; ++k) {
			double s = sincos_table[k][0];
			double c = sincos_table[k][1];
			Vec3 p = center + c * radius * u + s * radius * v;
			res.push_back(p);
		}

	}
}

