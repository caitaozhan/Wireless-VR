#pragma once

#include "K:\caitao\libgm-windows\libgm-windows\gm.h"

#include <vector>

class GMTracking {
public:
	GMTracking() {}
	~GMTracking() {}

	// Initializes the GMs. Returns false if any errors occur.
	bool init();

	// This function gets called everytime the VR gets new pose information.
	void update(float vr_x, float vr_y, float vr_z,
				float vr_qx, float vr_qy, float vr_qz, float vr_qw,
				float vr_yaw, float vr_pitch, float vr_roll);

	void computeMatrix();

private:
	// TX GMs (labeled horizontal and vertical)
	GM gm_h, gm_v;

	int init_gm_h, init_gm_v;

	// XYZ location of TX in global coordinates
	float tx_x, tx_y, tx_z;

	// The first non-zero XYZ from the VR headset is treated as the initial position of the VR headset
	bool vr_init_pos_set;
	float vr_init_x, vr_init_y, vr_init_z;

	// Let's us rotate from the VR coordinate system to the global frame of reference.
	std::vector<std::vector<float>> q_inv_rot_matrix;
};
