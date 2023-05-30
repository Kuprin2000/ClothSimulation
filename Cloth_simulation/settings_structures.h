#pragma once

struct ModelSettings
{
	int m_r_tree_min = 10;								// minimum number of children of r-tree node
	int m_r_tree_max = 20;								// maximum number of children of r-tree node

	float m_speed_damping_coefficient = 0.99f;			// damping coefficient of vertices speeds

	float m_gravity = 980.0f;							// gravitational acceleration

	int m_internal_constraints_threads_count = 4;		// number of threads we use for internal constraints
	int m_other_constraints_threads_count = 1;			// number of threads we use for collisions and user defined constraints
	int m_check_collisions_threads_count = 4;			// number of threads we use to check collisions
	int m_preferred_partitions_count = 25;				// how many partitions we want to get

	float m_max_collision_radius_for_cloth = 1.0f;		// shows at what distance from cloth program will create collision constraint
	float m_max_collision_radius_for_colliders = 2.0f;	// shows at what distance from object program will create collision constraint

	int m_iterations_count = 15;						// number of iterations
};

struct ConstraintsGraphSettings
{
	bool m_need_create_partitions = true;				// if need to create partitions for multithreaded execution
	int m_preferred_partitions_count = 0;				// how many partitions we want to get
	bool m_need_insert_phantoms = false;				// if need to insert phantom vertices
};