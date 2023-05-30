#pragma once
#include "cloth.h"
#include "collider.h"

namespace CheckCollisionsMultithreaded
{
	void checkSelfVertexTriangleCollision(Cloth& cloth, ConstraintsBuffers& buffers, float cloth_collision_radius, int constraint_id);

	void checkSelfEdgeEdgeCollision(Cloth& cloth, ConstraintsBuffers& buffers, float cloth_collision_radius, int constraint_id);

	void checkColliderVertexTriangleCollision(Cloth& cloth, ConstraintsBuffers& buffers,
		const Collider& colliders, float collider_collision_radius, int constraint_id);

	void checkColliderEdgeEdgeCollision(Cloth& cloth, ConstraintsBuffers& buffers,
		const Collider& colliders, float collider_collision_radius, int constraint_id);
}