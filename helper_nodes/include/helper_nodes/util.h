#ifndef UTILITY_H
#define UTILITY_H

#include <cmath>

namespace rough_cmp {
	inline bool equals(double x, double y, double threshold) { return (std::abs(x - y) < threshold); }
	inline bool lt_eq(double x, double y, double threshold) { return (x < y || equals(x, y, threshold)); }
	inline bool gt_eq(double x, double y, double threshold) { return (x > y || equals(x, y, threshold)); }
}

namespace circular_range {
	inline bool in_range(double min, double max, double val) {
		if(min > max && fabs(max - min) + min > 360.0) return ((val >= min && val < 360.0) || (val <= max && val >= 0.0));
		else return (val >= min && val <= max); // compares values on a [0, 360) and handles wrapping to 0
	}
	
	inline double average(double a, double b) {
		// shamelessly stolen from stackoverflow: 
		// https://stackoverflow.com/questions/491738/how-do-you-calculate-the-average-of-a-set-of-circular-data
		double diff = (fmod((a - b + 540.0), 360.0)) - 180.0; 
		return fmod((360.0 + b + (diff / 2.0)), 360.0);
	}

	inline double supplement(double angle) {
		if(angle >= 180.0) return (angle - 180.0);
		return (angle + 180.0);
	}
}

namespace geometric {
	static constexpr double pi = std::acos(-1);

	template<class point_t>
	inline double distance(point_t a, point_t b) { return std::hypot(b.x - a.x, b.y - a.y); }

	template<class point_t> // where a is your reference and b is your test point
	inline double angular_distance(point_t a, point_t b) { return std::atan2((b.y - a.y), (b.x - a.x)) * (180.0 / pi) + 180.0; };

	template<class point_t> // special case of average of 2 points
	inline point_t average(point_t a, point_t b) {
		point_t ret;
		ret.x = (a.x + b.x) / 2.0;
		ret.y = (a.y + b.y) / 2.0;
		return ret;
	}

	template<typename point_t_itr> // average of many points, in some sort of stl container
	inline typename point_t_itr::value_type n_average(point_t_itr start, point_t_itr end) {
		typename point_t_itr::value_type sum;
		sum.x = 0;
		sum.y = 0;
		int count = 0;

		for(point_t_itr current = start; current != end; ++current) {
			sum.x += current->x;
			sum.y += current->y;
			count++;
		}
	
		sum.x /= count;
		sum.y /= count;
	
		return sum;
	}
}

#endif
