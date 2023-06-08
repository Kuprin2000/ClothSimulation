#pragma once
#include "cloth.h"
#include "collider.h"

namespace CheckCollisionsMultithreaded
{
	void checkSelfVertexTriangleCollision(const Cloth& cloth, ConstraintsBuffers& buffers, float cloth_collision_radius,
		int constraint_id);

	void checkSelfEdgeEdgeCollision(const Cloth& cloth, ConstraintsBuffers& buffers, float cloth_collision_radius,
		int constraint_id);

	void checkColliderVertexTriangleCollision(const Cloth& cloth, ConstraintsBuffers& buffers,
		const Collider& colliders, float collider_collision_radius, int constraint_id);

	void checkColliderEdgeEdgeCollision(const Cloth& cloth, ConstraintsBuffers& buffers,
		const Collider& colliders, float collider_collision_radius, int constraint_id);
}