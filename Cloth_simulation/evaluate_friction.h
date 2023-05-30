#pragma once
#include "cloth.h"
#include "collider.h"

namespace EvaluateFrictionMultithreaded
{
	void evaluateSelfFriction(Cloth& cloth, const ConstraintsBuffers& buffers, int constraint_id, float time_delta);

	void evaluateColliderFriction(Cloth& cloth, const Collider& collider, const ConstraintsBuffers& buffers, int constraint_id, float time_delta);
}