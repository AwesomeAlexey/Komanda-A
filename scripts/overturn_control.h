#pragma once

double get_torque(double theta, double phi, double dtheta, double dphi);
double get_torque_sync(
	double theta, double phi, double dtheta, double dphi,
	double theta2, double phi2, double dtheta2, double dphi2
);
