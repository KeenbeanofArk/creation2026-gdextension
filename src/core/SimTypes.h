#pragma once
#include "Math3D.h"

// ============================================================
// Shared simulation data types used by multiple subsystems
// ============================================================

struct FoodItem {
	Vec3 position;
	float energy_value = 50.0f;
	bool active = true;
};
