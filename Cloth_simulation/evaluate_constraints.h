#pragma once
#include "cloth.h"
#include "collider.h"

namespace EvaluateConstraintsMultithread
{
	void evaluateStretch(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id);

	void evaluateRealisticStretch(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id);

	void evaluateBending(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id);

	void evaluateRealisticBending(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id);

	void evaluateSewing(Cloth& cloth, ConstraintsBuffers& buffers, float alpha_correction_coeff, int iteration_number, int constraint_id);
	
	void evaluatePhantom(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id);

	void evaluateFixedPosition(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id);

	void evaluateFixedAngle(Cloth& cloth, ConstraintsBuffers& buffers, float alpha_correction_coeff, int iteration_number, int constraint_id);

	void evaluateSelfVertexTriangleCollision(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id);

	void evaluateSelfVertexTriangleCollisionBetweenLayers(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id);

	void evaluateSelfEdgeEdgeCollision(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id);

	void evaluateColliderVertexTriangleCollision(Cloth& cloth, const Collider& colliders, ConstraintsBuffers& buffers, int constraint_id);

	void evaluateColliderEdgeEdgeCollision(Cloth& cloth, const Collider& colliders, ConstraintsBuffers& buffers, int constraint_id);
}