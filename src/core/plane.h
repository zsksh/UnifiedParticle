#pragma once
#include<core/vecg.h>
#include<vector>
#include<type_traits>
namespace UP {
	template <class T>
	class Plane {
	public:
		Plane() = default;
		Plane(T a, T b, T c, T d) : m_a{ a }, m_b{ b }, m_c{ c }, m_d{ d } {};
		Plane(const vec3g<T>& p, const vec3g<T>& n)
			: m_a{ n.x }, m_b{ n.y }, m_c{ n.z }, m_d{ -dot(p,n) } {}
		T side(const vec3g<T>& p) const {
			T result = m_a * p.x + m_b * p.y + m_c * p.z + m_d;
			return result;
		}
		T distance(const vec3g<T>& point) const {
			return abs(m_a * point.x + m_b * point.y + m_c * point.z + m_d)
				/ sqrt(m_a * m_a + m_b * m_b + m_c * m_c);
		}

		vec3g<T> normal() const { return vec3g<T>(m_a, m_b, m_c); }

	public:
		T m_a, m_b, m_c, m_d;
	};
}