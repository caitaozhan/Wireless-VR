#pragma once

#include "K:\caitao\libgm-windows\libgm-windows\gm.h"

#include <vector>
#include <array>

class GMTracking 
{
public:
	GMTracking() {}
	~GMTracking() {}

	// Initializes the GMs. Returns false if any errors occur.
	bool init();

	// This function gets called everytime the VR gets new pose information.
	void update(float vr_x, float vr_y, float vr_z,
				float vr_qx, float vr_qy, float vr_qz, float vr_qw,
				float vr_yaw, float vr_pitch, float vr_roll);
	
	// Caitao: decide to use std:array instead of std::vector because array's dimension is fixed to 3 (always 3 elements)
	std::array<std::array<float, 3>, 3> computeRotationMatrix(float vr_x, float vr_y, float vr_z);
	std::array<float, 3>                crossProduct(std::array<float, 3> vect_VR, std::array<float, 3> vect_GM);
	float                               dotProduct(std::array<float, 3> vect_VR, std::array<float, 3> vect_GM);

private:
	// TX GMs (labeled horizontal and vertical)
	GM tx_gm_h, tx_gm_v;
	GM rx_gm_h, rx_gm_v;

	int init_tx_gm_h, init_tx_gm_v;
	int init_rx_gm_h, init_rx_gm_v;

	// XYZ location of TX in global coordinates
	float tx_gm_x, tx_gm_y, tx_gm_z;

	// The first non-zero XYZ from the VR headset is treated as the initial position of the VR headset
	bool  vr_init_pos_set;
	float vr_init_x, vr_init_y, vr_init_z;

	// Let's us rotate from the VR coordinate system to the global frame of reference.
	std::vector<std::vector<float>> q_inv_rot_matrix;
};
