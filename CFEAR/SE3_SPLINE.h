#ifndef SE3_SPLINE_H
#define SE3_SPLINE_H

#include <vector>
#include <iostream>
using std::cout;
using std::endl;
#include <math.h>

#include "sophus/se3.hpp"
#include <Eigen/Dense>
using Sophus::SE3Base;
using Sophus::SE3;

const Eigen::Matrix4d C = (Eigen::Matrix4d() << 6.0 / 6.0, 0.0, 0.0, 0.0,
	5.0 / 6.0, 3.0 / 6.0, -3.0 / 6.0, 1.0 / 6.0,
	1.0 / 6.0, 3.0 / 6.0, 3.0 / 6.0, -2.0 / 6.0,
	0.0, 0.0, 0.0, 1.0 / 6.0).finished();

template<typename T>
Eigen::Matrix<T, 4, 1> spline_B(T u) {
	Eigen::Matrix<T, 4, 1> U(T(1.0), u, u * u, u * u * u);
	return C.cast<T>() * U;
}

template<typename T>
Eigen::Matrix<T, 4, 1> spline_Bprim(T u, double dt) {
	Eigen::Matrix<T, 4, 1> U(T(0.0), T(1.0), T(2) * u, T(3) * u * u);
	return C.cast<T>() * U / T(dt);
}

template<typename T>
Eigen::Matrix<T, 4, 1> spline_Bbis(T u, double dt) {
	double dt2 = dt * dt;
	Eigen::Matrix<T, 4, 1> U(T(0.0), T(0.0), T(2 / dt2), T(6) * u / T(dt2));
	return C.cast<T>() * U;
}


template<typename T>
class UniformSpline {
public:
	typedef SE3<T> SE3Type;
	typedef Eigen::Matrix<T, 4, 4> SE3DerivType;
	typedef Eigen::Matrix<T, 3, 1> Vec3;

	UniformSpline(const double dt = 1.0, const double offset = 0.0) : dt_(dt), offset_(offset) { };

	size_t num_knots() { return knots_.size(); }

	void add_knot(T* data);

	Eigen::Map<SE3Type> get_knot(size_t k) {
		//std::cout << Eigen::Map<SE3Type> (knots_[k]) << std::endl;
		return Eigen::Map<SE3Type>(knots_[k]);
	}

	double* get_knot_data(size_t i) {
		if (i < knots_.size()) {
			return knots_[i];
		}
		else {
			throw std::out_of_range("Knot does not exist");
		}
	}

	/** Evaluate spline (pose and its derivative)
	 * This gives the current pose and derivative of the spline.
	 * The Pose P = [R | t] is such that it moves a point from
	 * the spline coordinate frame, to the world coordinate frame.
	 * X_world = P X_spline
	 */
	void evaluate(T u, SE3Type& P, SE3DerivType& P_prim, SE3DerivType& P_bis);

	double get_dt() { return dt_; };

	double get_offset() { return offset_; };

	double min_time() {
		if (num_knots() > 0)
			return offset_ + dt_;
		else
			return 0.0;
	};

	double max_time() {
		if (num_knots() > 0)
			return offset_ + (dt_ * (num_knots() - 2));
		else
			return 0.0;
	};

	void zero_knots(size_t n) {
		knots_.clear();
		SE3Type identity;
		for (size_t i = 0; i < n; ++i) {
			double* data = new double[SE3Type::num_parameters];
			memcpy(data, identity.data(), sizeof(data[0]) * SE3Type::num_parameters);
			knots_.push_back(data);
		}
	}

protected:
	double dt_;
	double offset_;
	std::vector<T*> knots_;
};

template<typename T>
void UniformSpline<T>::add_knot(T* data) {
	knots_.push_back(data);
}

template<typename T>
void UniformSpline<T>::evaluate(T u, SE3Type& P, SE3DerivType& P_prim, SE3DerivType& P_bis) {
	typedef Eigen::Matrix<T, 4, 4> Mat4;
	typedef Eigen::Matrix<T, 4, 1> Vec4;
	typedef Eigen::Map<SE3Type> KnotMap;


	KnotMap P0 = Eigen::Map<SE3Type>(knots_[0]);

	P = P0;
	Vec4 B = spline_B(u);
	Vec4 B_prim = spline_Bprim(u, dt_);
	Vec4 B_bis = spline_Bbis(u, dt_);

	Mat4 A[3];
	Mat4 A_prim[3];
	Mat4 A_bis[3];

	for (int j : {1, 2, 3})
	{
		KnotMap knot1 = Eigen::Map<SE3Type>(knots_[j - 1]);
		KnotMap knot2 = Eigen::Map<SE3Type>(knots_[j]);
		typename SE3Type::Tangent omega = (knot1.inverse() * knot2).log();
		Mat4 omega_hat = SE3Type::hat(omega);
		SE3Type Aj = SE3Type::exp(B(j) * omega);
		P *= Aj;
		Mat4 Aj_prim = Aj.matrix() * omega_hat * B_prim(j);
		Mat4 Aj_bis = Aj_prim * omega_hat * B_prim(j) + Aj.matrix() * omega_hat * B_bis(j);
		A[j - 1] = Aj.matrix();
		A_prim[j - 1] = Aj_prim;
		A_bis[j - 1] = Aj_bis;
	}

	Mat4 M1 = A_prim[0] * A[1] * A[2] +
		A[0] * A_prim[1] * A[2] +
		A[0] * A[1] * A_prim[2];

	Mat4 M2 = A_bis[0] * A[1] * A[2] + A[0] * A_bis[1] * A[2] +
		A[0] * A[1] * A_bis[2] + T(2.0) * A_prim[0] * A_prim[1] * A[2] +
		T(2.0) * A_prim[0] * A[1] * A_prim[2] + T(2.0) * A[0] * A_prim[1] * A_prim[2];

	P_prim = P0.matrix() * M1;
	P_bis = P0.matrix() * M2;
}


#endif