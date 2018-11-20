#include "gm_tracking.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define GM_DATA_PATH "K:\\caitao\\libgm-windows\\libgm-windows\\gm_data"

// TODO Add simple classes to hold XYZ locations and 3x3 matrixes

bool GMTracking::init() {
	GM::verbose_level = 0;

	// Load GMs
	tx_gm_h.load(GM_DATA_PATH + std::string("\\tx_gm0.txt"));
	tx_gm_v.load(GM_DATA_PATH + std::string("\\tx_gm1.txt"));

	rx_gm_h.load(GM_DATA_PATH + std::string("\\rx_gm0.txt"));
	rx_gm_v.load(GM_DATA_PATH + std::string("\\rx_gm1.txt"));

	init_tx_gm_h = tx_gm_h.get();
	init_tx_gm_v = tx_gm_v.get();

	init_rx_gm_h = rx_gm_h.get();
	init_rx_gm_v = rx_gm_v.get();

	// Set TX GMs location (in meters)
	tx_gm_x = float(0.0);
	tx_gm_y = float(0.02);
	tx_gm_z = float(1.81); // distance between VR headset and TX

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
						float vr_yaw, float vr_pitch, float vr_roll) 
{
	if (vr_x == 0.0 && vr_y == 0.0 && vr_z == 0.0) 
	{
		return;
	}

	if (!vr_init_pos_set) 
	{
		vr_init_x = vr_x;
		vr_init_y = vr_y;
		vr_init_z = vr_z;
		vr_init_pos_set = true;
		///////////////////////////////////////////////////////////////////////////////////////////////////////

	}

	// TODO Find the receiver gm's new coordinates in the global system. For now just use the reported vr coordinates.
	// Calculate misalignment matrix
	std::array<std::array<float, 3>, 3> rotationMatrix = { { {0.9499,-0.1682,-0.2636}, {0.1682,0.9855,-0.0227}, {0.2636,-0.0227,0.9644} } };
	//std::array<std::array<float, 3>, 3> rotationMatrix = computeRotationMatrix(vr_x, vr_y, vr_z);
	// {0.9959,-0.0516,-0.0744}, {0.0516,0.9987,-0.0019}, {0.0744,-0.0019,0.9972}
	std::array<float, 3> vr_delta_xyz = { vr_x - vr_init_x, vr_y - vr_init_y, vr_z - vr_init_z };
	std::array<float, 3> GM_delta = { 0.0, 0.0, 0.0 };
	
	for (int i = 0; i < 3; ++i) 
	{
		for (int j = 0; j < 3; ++j)
		{
			GM_delta[i] += (rotationMatrix[i][j] * vr_delta_xyz[j]);
		}
	}
	float rx_gm_x = GM_delta[0];
	float rx_gm_y = GM_delta[1];
	float rx_gm_z = GM_delta[2];

	///////////////////////////////////////////
	//float rx_x = vr_x - vr_init_x;
	//float rx_y = vr_y - vr_init_y;
	//float rx_z = vr_z - vr_init_z;

	// THIS IS JUST TO AVOID WARNINGS FOR NOW. REMOVE THIS ONCE YPR IS USED
	float k = vr_yaw + vr_pitch + vr_roll + vr_qx + vr_qy + vr_qz + vr_qw;
	k++;
	
	// Find the new TX GM positions
	
	// Assume tx_gm_h and rx_gm_h modify only in X, and tx_gm_v and rx_gm_v modify in Y

	float tx_h_angle = float(atan((rx_gm_x - tx_gm_x - 0.177) / (tx_gm_z - rx_gm_z)) * 180.0 / M_PI);
	float tx_v_angle = float(atan((rx_gm_y - tx_gm_y + 0.02) / (tx_gm_z - rx_gm_z)) * 180.0 / M_PI);

	float rx_h_angle = float(atan((rx_gm_x - tx_gm_x - 0.177) / (tx_gm_z - rx_gm_z)) * 180.0 / M_PI);
	float rx_v_angle = float(atan((rx_gm_y - tx_gm_y + 0.02) / (tx_gm_z - rx_gm_z)) * 180.0 / M_PI);

	tx_gm_h.set(init_tx_gm_h + GM::degreeToGMUnit(tx_h_angle));
	tx_gm_v.set(init_tx_gm_v + GM::degreeToGMUnit(tx_v_angle));

	rx_gm_h.set(init_rx_gm_h + GM::degreeToGMUnit(rx_h_angle));
	rx_gm_v.set(init_rx_gm_v + GM::degreeToGMUnit(rx_v_angle));
}


std::array<std::array<float, 3>, 3> GMTracking::computeRotationMatrix(float vr_x, float vr_y, float vr_z)
{
	std::array<float, 3> vect_GM = { -1.0, 0.0, 0.0 }; // GM moving vector
	std::array<float, 3> VR_0    = { vr_x, vr_y, vr_z };  // Read VR position @t=0
	printf(" Calibration began, please move the headset... ");
	//usleep(5000);                        // give five sec for movement
	// TODO: x, y, z are changing!
	std::array<float, 3> VR_1 = { vr_x, vr_y, vr_z };  // read position after movement. // Read VR position @t=1
	std::array<float, 3> vect_VR_nonNorm = { VR_1[0] - VR_0[0], VR_1[1] - VR_0[1], VR_1[2] - VR_0[2] }; // VR moving vector
	// Now normalize VR vector
	float deter = sqrt(pow(vect_VR_nonNorm[0], 2) + pow(vect_VR_nonNorm[1], 2) + pow(vect_VR_nonNorm[2], 2));
	std::array<float, 3> vect_VR = { vect_VR_nonNorm[0] / deter, vect_VR_nonNorm[1] / deter, vect_VR_nonNorm[2] / deter }; // VR moving vector

	float product = dotProduct(vect_VR, vect_GM);
	std::array<float, 3> cross_P = crossProduct(vect_VR, vect_GM);

	// initialize matrix and vectors
	std::array<std::array<float, 3>, 3> rotm   = { { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} } };
	std::array<std::array<float, 3>, 3> skewM2 = { { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} } };
	std::array<std::array<float, 3>, 3> I      = { { {1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0} } };
	std::array<std::array<float, 3>, 3> skewM  = { { {0.0, -cross_P[2], cross_P[1]},{cross_P[2], 0.0, -cross_P[0]},{-cross_P[1], cross_P[0], 0.0} } }; //getting skew matrix

	for (int i = 0; i < 3; ++i)  // getting squared skew matrix
	{
		for (int j = 0; j < 3; ++j)
		{
			for (int k = 0; k < 3; ++k)
			{
				skewM2[i][j] += skewM[i][k] * skewM[k][j];
			}
		}
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rotm[i][j] = I[i][j] + skewM[i][j] + skewM2[i][j] / (product + 1);
		}
	}

	return rotm;
}

float GMTracking::dotProduct(std::array<float, 3> vect_VR, std::array<float, 3> vect_GM)
{
	float product = 0;
	for (int i = 0; i < 3; i++)   // Loop for calculate cot product
	{
		product = product + vect_VR[i] * vect_GM[i];
	}
	return product;
}

std::array<float, 3> GMTracking::crossProduct(std::array<float, 3> vect_VR, std::array<float, 3> vect_GM)
{
	std::array<float, 3> cross_P;

	cross_P[0] = vect_VR[1] * vect_GM[2] - vect_VR[2] * vect_GM[1];
	cross_P[1] = vect_VR[2] * vect_GM[0] - vect_VR[0] * vect_GM[2];
	cross_P[2] = vect_VR[0] * vect_GM[1] - vect_VR[1] * vect_GM[0];

	return cross_P;
}
