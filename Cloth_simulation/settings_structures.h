#pragma once

struct ModelSettings
{
	int m_r_tree_min = 10;								// minimum number of children of r-tree node
	int m_r_tree_max = 20;								// maximum number of children of r-tree node

	float m_speed_damping_coefficient = 0.99f;			// damping coefficient of vertices speeds

	float m_gravity = 980.0f;							// gravitational acceleration

	int m_preferred_partitions_count = 30;				// how many partitions we want to get

	float m_max_collision_radius_for_cloth = 1.0f;		// shows at what distance from cloth program will create collision constraint
	float m_max_collision_radius_for_colliders = 2.0f;	// shows at what distance from object program will create collision constraint

	int m_iterations_count = 20;						// number of iterations
};

struct ConstraintsGraphSettings
{
	int m_preferred_partitions_count = 0;				// how many partitions we want to get
	bool m_need_create_partitions = true;				// if need to create partitions for multithreaded execution
	bool m_need_insert_phantoms = false;				// if need to insert phantom vertices
};