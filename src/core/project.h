#include<core/geometry.h>
namespace UP {
	template<class vec3, class FT>
	bool constrained_disc_project(vec3& p, vec3& center, vec3& normal, FT& radius, vec3& fp, FT& dist) {
		// unconstrained results;
		fp = p - normal * dot((p - center), normal);
		auto d = distance(fp, center);
		if (d > radius) { //// out-of-disk
			vec3 dir = normalize(fp - center);
			fp = center + radius * dir;
			dist = distance(fp, p);
			return false;
		}
		dist = distance(fp, p);
		return true;

	}
}