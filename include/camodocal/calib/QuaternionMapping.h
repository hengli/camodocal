#ifndef QUATERNIONMAPPING_H
#define QUATERNIONMAPPING_H

#include <Eigen/Dense>
#include <iostream>

#include "DualQuaternion.h"

namespace camodocal
{

template<typename T>
Eigen::Quaternion<T> expq(const Eigen::Quaternion<T>& q)
{
	T a = q.vec().norm(); 
	T exp_w = exp(q.w());

	if (a == T(0))
	{
	    return Eigen::Quaternion<T>(exp_w, 0, 0, 0);
	}

	Eigen::Quaternion<T> res;
	res.w() = exp_w * T(cos(a));
	res.vec() = exp_w * T(sinc(a)) * q.vec();

	return res; 
}

template<typename T>
Eigen::Quaternion<T> logq(const Eigen::Quaternion<T>& q)
{
	T exp_w = q.norm();
	T w = log(exp_w);
	T a = acos(q.w() / exp_w); 	

	if (a == T(0))
	{
	    return Eigen::Quaternion<T>(w, T(0), T(0), T(0));
	}
	
	Eigen::Quaternion<T> res;
	res.w() = w;
	res.vec() = q.vec() / exp_w / (sin(a) / a);

	return res; 
}

}

#endif
