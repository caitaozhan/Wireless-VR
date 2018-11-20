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

	// start here
	// Calculate misalignment matrix
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Began moving headset at GM coordinat.
    float vect_GM[3] = {-1.0, 0.0, 0.0}; // GM moving vector
    float VR_0[3] = {vr_x, vr_y, vr_z}; // Read VR position @t=0
    printf(" Calibration began, please move the headset... ");
    //usleep(5000); // give five sec for movement
    float VR_1[3] = {vr_x, vr_y, vr_z};// read position after movement. // Read VR position @t=1
    float vect_VR_nonNorm[3] = {VR_1[0] - VR_0[0], VR_1[1] - VR_0[1], VR_1[2] - VR_0[2]}; // VR moving vector
    // Now normalize VR vector
    float deter = sqrt(pow(vect_VR_nonNorm[0], 2) + pow(vect_VR_nonNorm[1], 2) + pow(vect_VR_nonNorm[2], 2));
    float vect_VR[3] = {vect_VR_nonNorm[0]/deter, vect_VR_nonNorm[1]/deter, vect_VR_nonNorm[2]/deter}; // VR moving vector
    // find rotm with GM and VR vectors
    //**************************************************** calculate dot product **********************************//

    //**************************************************** calculate dot product **********************************//
    float dotProduct(float vect_VR[], float vect_GM[]) // calculate dot product
    {
        float product = 0;
        // Loop for calculate cot product
        for (int i = 0; i < 3; i++)
        {
                product = product + vect_VR[i] * vect_GM[i];
        }
        return product;
    }
    //**************************************************** calculate cross product ********************************//
    float crossProduct(float vect_VR[], float vect_GM[], float cross_P[])
    {
        float cross_P[3] = {0.0, 0.0, 0.0};

        cross_P[0] = vect_VR[1] * vect_GM[2] - vect_VR[2] * vect_GM[1];
        cross_P[1] = vect_VR[2] * vect_GM[0] - vect_VR[0] * vect_GM[2];
        cross_P[2] = vect_VR[0] * vect_GM[1] - vect_VR[1] * vect_GM[0];

        return cross_P[3];
    }
    //**************************************************** calculate cross product ********************************//
    //**************************************************** calculate for the rotation matrix rotm******************//
    // initialize matrix and vectors
	float rotm[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
	float skewM[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
	float skewM2[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
	float I[3][3] = {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}};
	float skewM[3][3] = {{0.0, -cross_P[2], cross_P[1]},{cross_P[2], 0.0, -cross_P[0]},{-cross_P[1], crossP[0], 0.0}}; //getting skew matrix

	for(int i = 0; i < 3; ++i)  // getting squared skew matrix
	{
        for(int j = 0; j < 3; ++j)
        {
            for(int k = 0; k < 3; ++k)
            {
                skewM2[i][j] += skewM[i][k] * skewM[k][j];
            }
        }
	}

	//calculate the rotm

	for (int i = 0; i < 3 ; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rotm[i][j] = I[i][j] + skewM[i][j] + skewM2[i][j]/(product + 1);
		}
	}

	//***************************************************** finished calculate the rotm*******************//
	// now rotm * (VR_t=1 - VR_t=0) = (GM_t=1 - GM_t=0)
	double vr_delta_xyz[3] = { vr_x - vr_init_x, vr_y - vr_init_y, vr_z - vr_init_z };
	double GM_delta[3] = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			GM_delta[i] += (rotm[i][j] * vr_delta_xyz[j]);
		}
	}
	double rx_x = GM_delta[0];
	double rx_y = GM_delta[1];
	double rx_z = GM_delta[2];

	//// to here ////

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

	float h_angle = float(atan((rx_x + 0.177) / (tx_z - rx_z)) * 180.0 / M_PI);
	float v_angle = float(atan((rx_y - 0.02) / (tx_z - rx_z)) * 180.0 / M_PI);

	gm_h.set(init_gm_h + GM::degreeToGMUnit(h_angle));
	gm_v.set(init_gm_v + GM::degreeToGMUnit(v_angle));
}
