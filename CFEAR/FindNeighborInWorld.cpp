/*-------------------------------------------------------------------------
Find a valid neighbor of i-th point in current_frame from keyframe.
Valid: within a radius 'grid_r' and an angle 'yaw_max'.
Return -1 when there are no vaild neighbors in keyframe.
-------------------------------------------------------------------------*/
#define _USE_MATH_DEFINES
#include <iostream>
#include "Functions.h"

using namespace std;

int FindNeighborInWorld(Scan current_frame, Scan keyframe, double* T_wt_data, double* T_wk_data, int i, double grid_r, double yaw_max) {

	const Eigen::Map<SE3d> T_wt((double*)&T_wt_data[0]); // Pose of current frame w.r.t. world
	const Eigen::Map<SE3d> T_wk((double*)&T_wk_data[0]); // Pose of keyframe w.r.t. world

	const Eigen::Vector3d mu_ti(current_frame.means[i]); // Coordinate of i-th point w.r.t. sensor at current frame
	const Eigen::Vector3d mu_wi = (T_wt.matrix() * mu_ti.homogeneous()).hnormalized(); // Coordinate of i-th point w.r.t. world

	const Eigen::Vector3d n_ti(current_frame.normals[i]); // Normal of i-th point w.r.t. sensor at current frame
	const Eigen::Vector3d n_wi = T_wt.matrix().block<3, 3>(0, 0) * n_ti; // Normal of i-th point w.r.t. world

	double dist = DBL_MAX, angle = DBL_MAX;

	int j = -1;
	for (int p = 0; p < keyframe.means.size(); p++) {

		const Eigen::Vector3d mu_kl(keyframe.means[p]); // Coordinate of p-th point w.r.t. sensor at keyframe
		const Eigen::Vector3d mu_wl = (T_wk.matrix() * mu_kl.homogeneous()).hnormalized(); // Coordinate of p-th point w.r.t. world

		const Eigen::Vector3d n_kl(keyframe.normals[p]); // Normal of p-th point w.r.t. sensor at keyframe
		const Eigen::Vector3d n_wl = T_wk.matrix().block<3, 3>(0, 0) * n_kl; // Normal of p-th point w.r.t. world

		double dist_new = hypot(mu_wi(0) - mu_wl(0), mu_wi(1) - mu_wl(1));
		double angle_new = acos(n_wi(0) * n_wl(0) + n_wi(1) * n_wl(1)); // Range: [0, PI]
		if (angle_new > M_PI / 2)
			angle_new = M_PI - angle_new; // Range: [0, PI/2]

		if (dist_new <= grid_r && angle_new <= yaw_max && dist_new < dist) {
			dist = dist_new;
			j = p;
		}
	}

	return j;
}