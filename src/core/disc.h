#pragma once
#include<core/vecg.h>
namespace UP {
	template <class FT>
	struct Disc {
		Disc() = default;
		void set_from(Plane<FT>& plane, FT* center) {
			m_normal[0] = plane.m_a;
			m_normal[1] = plane.m_b;
			m_normal[2] = plane.m_c;

			m_center[0] = center[0];
			m_center[1] = center[1];
			m_center[2] = center[2];
		}

		Disc(const std::array<FT, 3>& normal,
			const std::array<FT, 3>& center,
			FT radius)
			: m_normal(normal), m_center(center), m_radius(radius) {}
		std::array<FT, 3> m_normal;  // 3D normal vector
		std::array<FT, 3> m_center;  // 3D center
		FT m_radius;                 // Radius of the disc
	};
}