

#ifndef AABB_H
#define AABB_H


#include <math.h>
#include <algorithm>

#include "Vector3.h"

using std::min;
using std::max;
using std::endl;

namespace Potree{

class AABB{

public:
	Vector3<double> min;
	Vector3<double> max;
	Vector3<double> size;
	bool isInitialized;

	AABB(){
		min = Vector3<double>(std::numeric_limits<float>::max());
		max = Vector3<double>(-std::numeric_limits<float>::max());
		size = Vector3<double>(std::numeric_limits<float>::max());
		isInitialized = false;
	}

	AABB(const Vector3<double> &p) {
		min = p;
		max = p;
		size = max-min;
		isInitialized = true;
	}

	AABB(Vector3<double> min, Vector3<double> max){
		this->min = min;
		this->max = max;
		size = max-min;
		isInitialized = true;
	}

	bool isInside(const Vector3<double> &p){
		if(min.x <= p.x && p.x <= max.x){
			if(min.y <= p.y && p.y <= max.y){
				if(min.z <= p.z && p.z <= max.z){
					return true;
				}
			}
		}

		return false;
	}

	void update(const Vector3<double> &point){
		min.x = std::min(min.x, point.x);
		min.y = std::min(min.y, point.y);
		min.z = std::min(min.z, point.z);

		max.x = std::max(max.x, point.x);
		max.y = std::max(max.y, point.y);
		max.z = std::max(max.z, point.z);

		size = max - min;
		isInitialized = true;
	}

	void update(const AABB &aabb){
		update(aabb.min);
		update(aabb.max);
	}

	void makeCubic(){
		max = min + size.maxValue();
		size = max - min;
	}

	friend ostream &operator<<( ostream &output,  const AABB &value ){
		output << "min: " << value.min << endl;
		output << "max: " << value.max << endl;
		output << "size: " << value.size << endl;
		output << "initialized: " << std::boolalpha << value.isInitialized << endl;
		return output;
	}

};

}

#endif
