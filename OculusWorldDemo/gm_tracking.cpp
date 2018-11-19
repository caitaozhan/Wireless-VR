#include "gm_tracking.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define GM_DATA_PATH "K:\\caitao\\libgm-windows\\libgm-windows\\gm_data"

// TODO Add simple classes to hold XYZ locations and 3x3 matrixes

bool GMTracking::init() {
	GM::verbose_level = 0;

	// Load GMs
	gm_h.load(GM_DATA_PATH + std::string("\\gm0.txt"));
	gm_v.load(GM_DATA_PATH + std::string("\\gm1.txt"));

	init_gm_h = gm_h.get();
	init_gm_v = gm_v.get();

	// Set TX GMs location (in meters)
	tx_x = 0.0;
	tx_y = float(-0.03);
	tx_z = float(1.81); // distance between VR headset and TX

	vr_init_pos_set = false;
	vr_init_x = 0.0;
	vr_init_y = 0.0;
	vr_init_z = 0.0;

	q_inv_rot_matrix = std::vector<std::vector<float>>({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});

	return true;
}

float determinant(float matrix[3][3]) {
	float deter = 0.0;
	for (int i = 0; i < 3; ++i) {
		deter += matrix[0][i] * (matrix[1][(i + 1) % 3] * matrix[2][(i + 2) % 3] - matrix[1][(i + 2) % 3] * matrix[2][(i + 1) % 3]);
	}
	return deter;
}

void GMTracking::update(float vr_x, float vr_y, float vr_z,
						float vr_qx, float vr_qy, float vr_qz, float vr_qw,
						float vr_yaw, float vr_pitch, float vr_roll) {
	if (vr_x == 0.0 && vr_y == 0.0 && vr_z == 0.0) {
		return;
	}

	if (!vr_init_pos_set) {
		vr_init_x = vr_x;
		vr_init_y = vr_y;
		vr_init_z = vr_z;
		vr_init_pos_set = true;
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		

		////////////////////////////////////////////////////////////////////////////////////////////////////////
	}

	// TODO Find the receiver gm's new coordinates in the global system. For now just use the reported vr coordinates.
	// Calculate misalignment matrix
	double Matrix_Rot[3][3] = { {0.9499,-0.1682,-0.2636}, {0.1682,0.9855,-0.0227}, {0.2636,-0.0227,0.9644} };
	// {0.9959,-0.0516,-0.0744}, {0.0516,0.9987,-0.0019}, {0.0744,-0.0019,0.9972}
	double vr_delta_xyz[3] = { vr_x - vr_init_x, vr_y - vr_init_y, vr_z - vr_init_z };
	double GM_delta[3] = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			GM_delta[i] += (Matrix_Rot[i][j] * vr_delta_xyz[j]);
		}
	}
	double rx_x = GM_delta[0];
	double rx_y = GM_delta[1];
	double rx_z = GM_delta[2];

	///////////////////////////////////////////
	//float rx_x = vr_x - vr_init_x;
	//float rx_y = vr_y - vr_init_y;
	//float rx_z = vr_z - vr_init_z;

	// THIS IS JUST TO AVOID WARNINGS FOR NOW. REMOVE THIS ONCE YPR IS USED
	float k = vr_yaw + vr_pitch + vr_roll + vr_qx + vr_qy + vr_qz + vr_qw;
	k++;
	
	// Find the new TX GM positions
	

	// Assume TX is at (0.0, 0.0, tx_z) and initially poininting at (0, 0, 0).
	// Also assume that gm_h changes only in X dimension, and gm_v changes only in Y dimension.

	float h_angle = float(atan((rx_x - 0.18) / (tx_z - rx_z)) * 180.0 / M_PI);
	float v_angle = float(atan((rx_y - 0.02) / (tx_z - rx_z)) * 180.0 / M_PI);

	gm_h.set(init_gm_h + GM::degreeToGMUnit(h_angle));
	gm_v.set(init_gm_v + GM::degreeToGMUnit(v_angle));
}
