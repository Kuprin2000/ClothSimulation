// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "evaluate_constraints.h"
#include "defines.h"

constexpr static const float C_TOLERANCE = FLT_EPSILON;

void EvaluateConstraintsMultithread::evaluateSewing(Cloth& cloth, ConstraintsBuffers& buffers, float alpha_correction_coeff, int iteration_number, int constraint_id)
{
	// this is original constraint that works in two steps:
	// 1) move vertices to each other
	// 2) when vertices are close enough, join them into one vertex

	// get constraint data
	// SEW_VERTICES: 2 - we sew them
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SEW_VERTICES];

	// here we can deal with phantom vertices
	const int real_vertices_count = cloth.getRealVerticesCount();
	uint32_t constraint_vertices[vertices_count] = { 0u };
	glm::vec3* vertices_test_coords[vertices_count] = { nullptr };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = ((int)raw_constraint_vertices[i] >= real_vertices_count) ? raw_constraint_vertices[i] : host_vertices[raw_constraint_vertices[i]];
		vertices_test_coords[i] = &cloth.getTestCoords(constraint_vertices[i]);
	}

	// vertices are sewed already
	if (constraint_vertices[0] == constraint_vertices[1])
	{
		return;
	}

	// SEW_VERTICES: 1 - opposite stiffness, 1 - distance to join vertices, 1 - lambda for XPBD
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data for constraint
	const float overline_alpha = constraint_data[0] * alpha_correction_coeff;
	const float distance_to_join_vertices = constraint_data[1];

	float& lambda_accumulator = constraint_data[2];
	lambda_accumulator = iteration_number ? lambda_accumulator : 0.0f;

	glm::vec3 vector_between_vertices = *vertices_test_coords[0] - *vertices_test_coords[1];
	const float distance_between_vertices = glm::length(vector_between_vertices);

	// if vertices are cloth enough set the second vertex to be host of the first one
	if (distance_between_vertices < distance_to_join_vertices)
	{
		cloth.setHostOrOriginalVertex(constraint_vertices[0], constraint_vertices[1]);
		return;
	}

	const float C = distance_between_vertices - distance_to_join_vertices;

	// we don't need to perform corrections
	if (fabsf(C) < C_TOLERANCE)
	{
		return;
	}

	// calc corrections
	vector_between_vertices /= distance_between_vertices;
	const glm::vec2 opposite_masses = { cloth.getOppositeMass(host_vertices[constraint_vertices[0]]), cloth.getOppositeMass(host_vertices[constraint_vertices[1]]) };
	const float delta_lambda = (-C - overline_alpha * lambda_accumulator) / (opposite_masses[0] + opposite_masses[1] + overline_alpha);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SEW_VERTICES]] =
	{
		delta_lambda * opposite_masses[0] * vector_between_vertices,
		-delta_lambda * opposite_masses[1] * vector_between_vertices

	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SEW_VERTICES]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1];
	}
#endif

	// apply corrections
	// derivative[0] is vector_between_vertices
	// derivative[1] is -vector_between_vertices
	* vertices_test_coords[0] += delta_lambda * opposite_masses[0] * vector_between_vertices;
	*vertices_test_coords[1] += -delta_lambda * opposite_masses[1] * vector_between_vertices;

	// update lambda for XPBD
	lambda_accumulator += delta_lambda;
}

void EvaluateConstraintsMultithread::evaluateStretch(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id)
{
	// you can read about this stretch constraint in
	// "Position-Based Simulation Methods in Computer Graphics", p.6

	// get constraint data
	// STRETCH: 2 - we control distance between them
	ConstraintsBuffers& buffers = cloth.getInternalConstraints();
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::STRETCH];

	// here we can deal with phantom vertices
	const int real_vertices_count = cloth.getRealVerticesCount();
	uint32_t constraint_vertices[vertices_count] = { 0u };
	glm::vec3* vertices_test_coords[vertices_count] = { nullptr };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = ((int)raw_constraint_vertices[i] >= real_vertices_count) ? raw_constraint_vertices[i] : host_vertices[raw_constraint_vertices[i]];
		vertices_test_coords[i] = &cloth.getTestCoords(constraint_vertices[i]);
	}

	// STRETCH: 1 - recommended distance, 1 - opposite stiffness, 1 - lambda for XPBD
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data for constraint
	const float optimal_distance_between_vertices = constraint_data[0];
	const float overline_alpha = constraint_data[1] * alpha_correction_coeff;

	float& lambda_accumulator = constraint_data[2];
	lambda_accumulator = iteration_number ? lambda_accumulator : 0.0f;

	glm::vec3 vector_between_vertices = *vertices_test_coords[0] - *vertices_test_coords[1];
	const float distance_between_vertices = glm::length(vector_between_vertices);
	const float C = distance_between_vertices - optimal_distance_between_vertices;

	// we don't need to perform corrections
	if (fabsf(C) < C_TOLERANCE)
	{
		return;
	}

	// calc corrections
	vector_between_vertices /= distance_between_vertices;
	const glm::vec2 opposite_masses = { cloth.getOppositeMass(host_vertices[constraint_vertices[0]]), cloth.getOppositeMass(host_vertices[constraint_vertices[1]]) };
	const float delta_lambda = (-C - overline_alpha * lambda_accumulator) / (opposite_masses[0] + opposite_masses[1] + overline_alpha);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::STRETCH]] =
	{
		delta_lambda * opposite_masses[0] * vector_between_vertices,
		-delta_lambda * opposite_masses[1] * vector_between_vertices

	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::STRETCH]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1];
	}
#endif

	// apply corrections
	// derivative[0] is vector_between_vertices
	// derivative[1] is -vector_between_vertices
	* vertices_test_coords[0] += delta_lambda * opposite_masses[0] * vector_between_vertices;
	*vertices_test_coords[1] += -delta_lambda * opposite_masses[1] * vector_between_vertices;

	// update lambda for XPBD
	lambda_accumulator += delta_lambda;
}

void EvaluateConstraintsMultithread::evaluateRealisticStretch(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id)
{
	// you can read about this constraint in
	// "Position-Based Simulation of Continuous Materials", p.10

	// get constraint data
	// REALISTIC_STRETCH: 3 - triangle controlled by the constraint
	ConstraintsBuffers& buffers = cloth.getInternalConstraints();
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::REALISTIC_STRETCH];

	// here we can deal with phantom vertices
	const int real_vertices_count = cloth.getRealVerticesCount();
	uint32_t constraint_vertices[vertices_count] = { 0u };
	glm::vec3* vertices_test_coords[vertices_count] = { nullptr };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = ((int)raw_constraint_vertices[i] >= real_vertices_count) ? raw_constraint_vertices[i] : host_vertices[raw_constraint_vertices[i]];
		vertices_test_coords[i] = &cloth.getTestCoords(constraint_vertices[i]);
	}

	// REALISTIC_STRETCH: 4 - to specify reference inverted shape matrix, 1 - reference triangle square, 1 - opposite stiffness, 1 - lambda for XPBD
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data
	// TIP: second constructor parameter is [0][1]
	const glm::mat2x2 reference_inverse_shape_matrix(constraint_data[0], constraint_data[1], constraint_data[2], constraint_data[3]);
	const float reference_triangle_square = constraint_data[4];
	const float overline_alpha = constraint_data[5] * alpha_correction_coeff;

	float& lambda_accumulator = constraint_data[6];
	lambda_accumulator = iteration_number ? lambda_accumulator : 0.0f;


	// perform calculations to find strain energy of the cloth triangle
	glm::vec3 first_local_basis_vector;
	glm::vec3 second_local_basis_vector;
	MathUtils::triangleLocalBasis(*vertices_test_coords[0], *vertices_test_coords[1], *vertices_test_coords[2], first_local_basis_vector, second_local_basis_vector);

	// you can read how to find triangle shape matrix here
	// https://www.continuummechanics.org/finiteelementmapping.html

	const glm::mat2x2 current_shape_matrix = MathUtils::triangleShapeMatrix(*vertices_test_coords[0], *vertices_test_coords[1], *vertices_test_coords[2]);

	const glm::mat2x2 deformation_gradient = current_shape_matrix * reference_inverse_shape_matrix;

	// you can read how to transform symmetric tensor to matrix here
	// http://imagine.inrialpes.fr/people/Francois.Faure/htmlCourses/FiniteElements.html
	glm::mat2x2 green_strain_tensor_as_matrix = glm::transpose(deformation_gradient) * deformation_gradient;
	green_strain_tensor_as_matrix -= glm::mat2x2(1.0f, 0.0f, 0.0f, 1.0f);
	green_strain_tensor_as_matrix *= 0.5f;

	const glm::vec3 green_strain_tensor_as_vec(green_strain_tensor_as_matrix[0][0], green_strain_tensor_as_matrix[1][1], green_strain_tensor_as_matrix[1][0]);
	const glm::vec3 stress_as_vec = cloth.getPartElasticityTensor(cloth.getVertexPartId(constraint_vertices[0])) * green_strain_tensor_as_vec;

	const glm::mat2x2 stress_as_mat(stress_as_vec[0], stress_as_vec[2], stress_as_vec[2], stress_as_vec[1]);
	const float scalar_strain_energy_density = 0.5f * MathUtils::traceOfMatrix22(glm::transpose(green_strain_tensor_as_matrix) * stress_as_mat);

	// energy is C in this case, but we check value of scalar_strain_energy_density
	// we don't need to perform corrections
	if (fabsf(scalar_strain_energy_density) < C_TOLERANCE)
	{
		return;
	}

	const float energy = scalar_strain_energy_density * reference_triangle_square;

	// find corrections
	const glm::mat2x2 resulting_matrix = deformation_gradient * stress_as_mat * glm::transpose(reference_inverse_shape_matrix) * reference_triangle_square;

	glm::vec2 derivatives_in_local_space[3] = { glm::vec2(0.0f, 0.0f) };
	derivatives_in_local_space[0] = { resulting_matrix[0][0], resulting_matrix[0][1] };
	derivatives_in_local_space[1] = { resulting_matrix[1][0], resulting_matrix[1][1] };
	derivatives_in_local_space[2][0] = -derivatives_in_local_space[0][0] - derivatives_in_local_space[1][0];
	derivatives_in_local_space[2][1] = -derivatives_in_local_space[0][1] - derivatives_in_local_space[1][1];

	glm::vec3 derivatives[3] =
	{
		MathUtils::vectorFromLocalToGlobal(derivatives_in_local_space[0], first_local_basis_vector, second_local_basis_vector),
		MathUtils::vectorFromLocalToGlobal(derivatives_in_local_space[1], first_local_basis_vector, second_local_basis_vector),
		MathUtils::vectorFromLocalToGlobal(derivatives_in_local_space[2], first_local_basis_vector, second_local_basis_vector)
	};

	// norms of derivatives
	const glm::vec3 derivatives_norms =
	{
		glm::dot(derivatives[0],derivatives[0]),
		glm::dot(derivatives[1],derivatives[1]),
		glm::dot(derivatives[2],derivatives[2])
	};

	// lambda
	const glm::vec3 opposite_masses =
	{
		cloth.getOppositeMass(host_vertices[constraint_vertices[0]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[1]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[2]])
	};
	const float delta_lambda = (-energy - overline_alpha * lambda_accumulator) / (glm::dot(derivatives_norms, opposite_masses) + overline_alpha);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::REALISTIC_STRETCH]] =
	{
		delta_lambda * opposite_masses[0] * derivatives[0],
		delta_lambda * opposite_masses[1] * derivatives[1],
		delta_lambda * opposite_masses[2] * derivatives[2]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::REALISTIC_STRETCH]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1]),
		glm::length(deltas[2])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH || deltas_lengths[2] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1] + deltas_lengths[2];
	}
#endif

	// apply corrections
	* vertices_test_coords[0] += delta_lambda * opposite_masses[0] * derivatives[0];
	*vertices_test_coords[1] += delta_lambda * opposite_masses[1] * derivatives[1];
	*vertices_test_coords[2] += delta_lambda * opposite_masses[2] * derivatives[2];

	// update lambda for XPBD
	lambda_accumulator += delta_lambda;
}

void EvaluateConstraintsMultithread::evaluateBending(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id)
{
	// you can read about this constraint in
	// "Position-Based Simulation Methods in Computer Graphics", p.10

	// get constraint data
	// BEND: 4 - two triangles with common edge
	ConstraintsBuffers& buffers = cloth.getInternalConstraints();
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::BEND];

	// here we can deal with phantom vertices
	// here we have to create a copy of test coords
	const int real_vertices_count = cloth.getRealVerticesCount();
	uint32_t constraint_vertices[vertices_count] = { 0u };
	glm::vec3 vertices_test_coords_copy[vertices_count] = { glm::vec3(0.0f, 0.0f, 0.0f) };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = ((int)raw_constraint_vertices[i] >= real_vertices_count) ? raw_constraint_vertices[i] : host_vertices[raw_constraint_vertices[i]];
		vertices_test_coords_copy[i] = cloth.getTestCoords(constraint_vertices[i]);
	}

	// BEND: 1 - initial angle between triangles, 1 - opposite stiffness, 1 - lambda for XPBD
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data
	const float initial_angle = constraint_data[0];
	const float overline_alpha = constraint_data[1] * alpha_correction_coeff;

	float& lambda_accumulator = constraint_data[2];
	lambda_accumulator = iteration_number ? lambda_accumulator : 0.0f;

	// first triangle vertices are [1], [0], [2]
	// second triangle vertices are [0], [1], [3]

	// normalize all coords
	vertices_test_coords_copy[1] -= vertices_test_coords_copy[0];
	vertices_test_coords_copy[2] -= vertices_test_coords_copy[0];
	vertices_test_coords_copy[3] -= vertices_test_coords_copy[0];
	vertices_test_coords_copy[0] = { 0.0f, 0.0f, 0.0f };

	const glm::vec3 triangle_a_normal = glm::normalize(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[2]));
	const glm::vec3 triangle_b_normal = glm::normalize(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[3]));
	const float current_angle_cos = glm::dot(triangle_a_normal, triangle_b_normal);

	const float C = MathUtils::safeAcos(current_angle_cos) - initial_angle;

	// we don't need to perform corrections
	if (fabsf(C) < C_TOLERANCE || current_angle_cos + 1.0f < FLT_EPSILON)
	{
		return;
	}

	// find corrections

	glm::vec3 derivatives[4] = { glm::vec3(0.0f, 0.0f, 0.0f) };

	derivatives[1] = -(glm::cross(vertices_test_coords_copy[2], triangle_b_normal) + glm::cross(triangle_a_normal, vertices_test_coords_copy[2]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[2]));
	derivatives[1] -= (glm::cross(vertices_test_coords_copy[3], triangle_a_normal) + glm::cross(triangle_b_normal, vertices_test_coords_copy[3]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[3]));

	derivatives[2] = (glm::cross(vertices_test_coords_copy[1], triangle_b_normal) + glm::cross(triangle_a_normal, vertices_test_coords_copy[1]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[2]));

	derivatives[3] = (glm::cross(vertices_test_coords_copy[1], triangle_a_normal) + glm::cross(triangle_b_normal, vertices_test_coords_copy[1]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[3]));

	derivatives[0] = -derivatives[1] - derivatives[2] - derivatives[3];

	const float coefficient = 1.0f / sqrtf(1.0f - current_angle_cos * current_angle_cos);
	derivatives[0] *= coefficient;
	derivatives[1] *= coefficient;
	derivatives[2] *= coefficient;
	derivatives[3] *= coefficient;

	// norms of derivatives
	glm::vec4 derivatives_norms =
	{
		glm::dot(derivatives[0],derivatives[0]),
		glm::dot(derivatives[1],derivatives[1]),
		glm::dot(derivatives[2],derivatives[2]),
		glm::dot(derivatives[3],derivatives[3])
	};

	glm::vec4 opposite_masses =
	{
		cloth.getOppositeMass(host_vertices[constraint_vertices[0]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[1]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[2]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[3]]) };

	const float delta_lambda = (-C - overline_alpha * lambda_accumulator) / (glm::dot(opposite_masses, derivatives_norms) + overline_alpha);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::BEND]] =
	{
		delta_lambda * opposite_masses[0] * derivatives[0],
		delta_lambda * opposite_masses[1] * derivatives[1],
		delta_lambda * opposite_masses[2] * derivatives[2],
		delta_lambda * opposite_masses[3] * derivatives[3]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::BEND]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1]),
		glm::length(deltas[2]),
		glm::length(deltas[3])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH || deltas_lengths[2] > MAX_LENGTH || deltas_lengths[3] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1] + deltas_lengths[2] + deltas_lengths[3];
	}
#endif

	// apply corrections
	// now we need pointers but not copies
	cloth.getTestCoords(constraint_vertices[0]) += delta_lambda * opposite_masses[0] * derivatives[0];
	cloth.getTestCoords(constraint_vertices[1]) += delta_lambda * opposite_masses[1] * derivatives[1];
	cloth.getTestCoords(constraint_vertices[2]) += delta_lambda * opposite_masses[2] * derivatives[2];
	cloth.getTestCoords(constraint_vertices[3]) += delta_lambda * opposite_masses[3] * derivatives[3];

	// update lambda for XPBD
	lambda_accumulator += delta_lambda;
}

void EvaluateConstraintsMultithread::evaluateRealisticBending(Cloth& cloth, float alpha_correction_coeff, int iteration_number, int constraint_id)
{
	// get constraint data
	// REALISTIC_BEND: 4 - two triangles with common edge
	ConstraintsBuffers& buffers = cloth.getInternalConstraints();
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::REALISTIC_BEND];

	// here we can deal with phantom vertices
	const int real_vertices_count = cloth.getRealVerticesCount();
	uint32_t constraint_vertices[vertices_count] = { 0u };
	glm::vec3* vertices_test_coords[vertices_count] = { nullptr };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = ((int)raw_constraint_vertices[i] >= real_vertices_count) ? raw_constraint_vertices[i] : host_vertices[raw_constraint_vertices[i]];
		vertices_test_coords[i] = &cloth.getTestCoords(constraint_vertices[i]);
	}

	// REALISTIC_BEND: 16 - Hessian energy matrix, 1 - opposite stiffness, 1 - lambda for XPBD
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data
	const glm::mat4x4 energy_matrix =
	{
		constraint_data[0],
		constraint_data[1],
		constraint_data[2],
		constraint_data[3],
		constraint_data[4],
		constraint_data[5],
		constraint_data[6],
		constraint_data[7],
		constraint_data[8],
		constraint_data[9],
		constraint_data[10],
		constraint_data[11],
		constraint_data[12],
		constraint_data[13],
		constraint_data[14],
		constraint_data[15],
	};

	const float overline_alpha = constraint_data[16] * alpha_correction_coeff;

	float& lambda_accumulator = constraint_data[17];
	lambda_accumulator = iteration_number ? lambda_accumulator : 0.0f;

	// find energy
	float C = 0.0f;
	for (uint8_t i = 0; i < 4u; ++i)
	{
		for (uint8_t j = 0; j < 4u; ++j)
		{
			C += energy_matrix[j][i] * glm::dot(*vertices_test_coords[i], *vertices_test_coords[j]);
		}
	}
	C *= 0.5f;

	// we don't need to perform corrections
	if (fabsf(C) < C_TOLERANCE)
	{
		return;
	}

	// find corrections

	// derivatives
	glm::vec3 derivatives[4] = { {0.0f, 0.0f, 0.0f} };
	for (uint8_t i = 0; i < 4; ++i)
	{
		derivatives[i] += energy_matrix[0][i] * *vertices_test_coords[0] + energy_matrix[1][i] * *vertices_test_coords[1] +
			energy_matrix[2][i] * *vertices_test_coords[2] + energy_matrix[3][i] * *vertices_test_coords[3];
	}

	// norms of derivatives
	const glm::vec4 derivatives_norms =
	{
		glm::dot(derivatives[0],derivatives[0]),
		glm::dot(derivatives[1],derivatives[1]),
		glm::dot(derivatives[2],derivatives[2]),
		glm::dot(derivatives[3],derivatives[3])
	};

	// lambda
	const glm::vec4 opposite_masses = { cloth.getOppositeMass(host_vertices[constraint_vertices[0]]), cloth.getOppositeMass(host_vertices[constraint_vertices[1]]),
										cloth.getOppositeMass(host_vertices[constraint_vertices[2]]), cloth.getOppositeMass(host_vertices[constraint_vertices[3]]) };

	const float division_coeff = glm::dot(opposite_masses, derivatives_norms);
	const float delta_lambda = (-C - overline_alpha * lambda_accumulator) / (division_coeff + overline_alpha);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::REALISTIC_BEND]] =
	{
		delta_lambda * opposite_masses[0] * derivatives[0],
		delta_lambda * opposite_masses[1] * derivatives[1],
		delta_lambda * opposite_masses[2] * derivatives[2],
		delta_lambda * opposite_masses[3] * derivatives[3]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::REALISTIC_BEND]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1]),
		glm::length(deltas[2]),
		glm::length(deltas[3])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH || deltas_lengths[2] > MAX_LENGTH || deltas_lengths[3] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1] + deltas_lengths[2] + deltas_lengths[3];
	}
#endif

	// apply corrections
	* vertices_test_coords[0] += delta_lambda * opposite_masses[0] * derivatives[0];
	*vertices_test_coords[1] += delta_lambda * opposite_masses[1] * derivatives[1];
	*vertices_test_coords[2] += delta_lambda * opposite_masses[2] * derivatives[2];
	*vertices_test_coords[3] += delta_lambda * opposite_masses[3] * derivatives[3];

	// update lambda of XPBD
	lambda_accumulator += delta_lambda;
}

void EvaluateConstraintsMultithread::evaluatePhantom(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id)
{
	// you can read about this constraint in
	// "Scalable Partitioning for Parallel Position Based Dynamics", p.5

	// PHANTOM_VERTICES: 25 - this is maximum vertices per phantom
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int max_vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::PHANTOM_VERTICES];
	const int real_vertices_count = (int)roundf(*buffers.getConstraintData(constraint_id));

	// all vertices here are phantoms
	glm::vec3* vertices_test_coords[max_vertices_count] = { nullptr };
	for (int i = 0; i < real_vertices_count; ++i)
	{
		vertices_test_coords[i] = &cloth.getTestCoords(raw_constraint_vertices[i]);
	}

	// PHANTOM_VERTICES: 1 - number of vertices
	// we used this data before

	// find barycenter
	glm::vec3 coords_accumulator = { 0.0f, 0.0f, 0.0f };
	for (int i = 0; i < real_vertices_count; ++i)
	{
		coords_accumulator += *vertices_test_coords[i];
	}
	coords_accumulator /= (float)real_vertices_count;

	// apply new position
	for (int i = 0; i < real_vertices_count; ++i)
	{
		*vertices_test_coords[i] = coords_accumulator;
	}
}

void EvaluateConstraintsMultithread::evaluateFixedPosition(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id)
{
	// this is original constraints. It just sets specified position of the vertex

	// get constraint data
	// FIXED_POSITION: 1 - one vertex
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	glm::vec3* vertex_test_coords = &cloth.getTestCoords(host_vertices[raw_constraint_vertices[0]]);

	// FIXED_POSITION: 3 - position of a vertex
	const float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data
	const glm::vec3 specified_vertex_position = { constraint_data[0], constraint_data[1], constraint_data[2] };

	// move vertex to the specific position
	*vertex_test_coords = specified_vertex_position;
}

void EvaluateConstraintsMultithread::evaluateFixedAngle(Cloth& cloth, ConstraintsBuffers& buffers, float alpha_correction_coeff, int iteration_number, int constraint_id)
{
	// this is original constraint based on bending constraint
	// to set fixed angle between two triangles with the common edge

	// get constraint data
	// FIXED_ANGLE: 4 - two triangles with common edge
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::FIXED_ANGLE];

	// all vertices here are real
	uint32_t constraint_vertices[vertices_count] = { 0u };
	glm::vec3 vertices_test_coords_copy[vertices_count] = { glm::vec3(0.0f, 0.0f, 0.0f) };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = host_vertices[raw_constraint_vertices[i]];
		vertices_test_coords_copy[i] = cloth.getTestCoords(constraint_vertices[i]);
	}

	// FIXED_ANGLE: 1 - optimal angle between triangles, 1 - opposite stiffness, 1 - lambda for XPBD
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// first triangle vertices are [1], [0], [2]
	// second triangle vertices are [0], [1], [3]
	// both triangles should go around counterclockwise

	// get special data
	const float optimal_angle = constraint_data[0];
	const float overline_alpha = constraint_data[1] * alpha_correction_coeff;

	float& lambda_accumulator = constraint_data[2];
	lambda_accumulator = iteration_number ? lambda_accumulator : 0.0f;

	const float initial_angle = MathUtils::PI;

	// define common edge of the triangles and their free vertices
	const glm::vec3 common_edge_vertex_1 = vertices_test_coords_copy[0];
	const glm::vec3 common_edge_vertex_2 = vertices_test_coords_copy[1];
	glm::vec3 free_vertex_in_triangle_a = vertices_test_coords_copy[2];
	glm::vec3 free_vertex_in_triangle_b = vertices_test_coords_copy[3];

	// now we have to hack original bending constraint to make it bend cloth to get right angle
	// we virtually move free vertices of triangles to have wrong angle and then put these fake
	// positions to bending constraint

	glm::vec3 edge_vector = glm::normalize(common_edge_vertex_2 - common_edge_vertex_1);

	const glm::vec3 closest_point_on_edge_for_free_vertex_a = MathUtils::closestPointOnLine(free_vertex_in_triangle_a, common_edge_vertex_1, edge_vector);
	const glm::vec3 closest_point_on_edge_for_free_vertex_b = MathUtils::closestPointOnLine(free_vertex_in_triangle_b, common_edge_vertex_1, edge_vector);

	glm::vec3 shortest_vector_from_edge_to_free_vertex_a = free_vertex_in_triangle_a - closest_point_on_edge_for_free_vertex_a;
	glm::vec3 shortest_vector_from_edge_to_free_vertex_b = free_vertex_in_triangle_b - closest_point_on_edge_for_free_vertex_b;

	const float angle_direction_indicator = glm::sign(MathUtils::tripleProduct(shortest_vector_from_edge_to_free_vertex_a,
		shortest_vector_from_edge_to_free_vertex_b, edge_vector));

	// free vertices moved to the direction of normals
	// or free vertices moved to the direction opposite to normals
	float current_angle = MathUtils::dihedralAngleBetweenTriangles(vertices_test_coords_copy[0],
		vertices_test_coords_copy[1], vertices_test_coords_copy[2], vertices_test_coords_copy[3]);
	current_angle = MathUtils::PI + angle_direction_indicator * current_angle;

	// we don't have to perform corrections
	if (fabsf(current_angle - optimal_angle) < C_TOLERANCE)
	{
		return;
	}

	// hack original constraint
	const float angle_delta_for_vectors = (optimal_angle - current_angle) * 0.5f;
	const float vector_a_rotation_angle_to_create_fake_a = angle_delta_for_vectors;
	const float vector_b_rotation_angle_to_create_fake_b = -angle_delta_for_vectors;

	const float angle_to_rotate_to_rest_state = -(MathUtils::PI - current_angle) * 0.5f;
	const float angle_to_rotate_vector_a_to_rest_state = angle_to_rotate_to_rest_state;
	const float angle_to_rotate_vector_b_to_rest_state = -angle_to_rotate_to_rest_state;

	shortest_vector_from_edge_to_free_vertex_a = MathUtils::rotateVectorAroundDirection(shortest_vector_from_edge_to_free_vertex_a, edge_vector,
		angle_to_rotate_vector_a_to_rest_state + vector_a_rotation_angle_to_create_fake_a);
	shortest_vector_from_edge_to_free_vertex_b = MathUtils::rotateVectorAroundDirection(shortest_vector_from_edge_to_free_vertex_b, edge_vector,
		angle_to_rotate_vector_b_to_rest_state + vector_b_rotation_angle_to_create_fake_b);

	vertices_test_coords_copy[2] = closest_point_on_edge_for_free_vertex_a + shortest_vector_from_edge_to_free_vertex_a;
	vertices_test_coords_copy[3] = closest_point_on_edge_for_free_vertex_b + shortest_vector_from_edge_to_free_vertex_b;
	free_vertex_in_triangle_a = closest_point_on_edge_for_free_vertex_a + shortest_vector_from_edge_to_free_vertex_a;
	free_vertex_in_triangle_b = closest_point_on_edge_for_free_vertex_b + shortest_vector_from_edge_to_free_vertex_b;

	// first triangle vertices are [1], [0], [2]
	// second triangle vertices are [0], [1], [3]

	// perform original constraint

	// normalize all coords
	vertices_test_coords_copy[1] -= vertices_test_coords_copy[0];
	vertices_test_coords_copy[2] -= vertices_test_coords_copy[0];
	vertices_test_coords_copy[3] -= vertices_test_coords_copy[0];
	vertices_test_coords_copy[0] = { 0.0f, 0.0f, 0.0f };

	const glm::vec3 triangle_a_normal = glm::normalize(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[2]));
	const glm::vec3 triangle_b_normal = glm::normalize(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[3]));
	const float current_angle_cos = glm::dot(triangle_a_normal, triangle_b_normal);

	const float C = MathUtils::safeAcos(current_angle_cos) - initial_angle;

	// we don't need to perform corrections
	if (fabsf(C) < C_TOLERANCE || current_angle_cos + 1.0f < FLT_EPSILON)
	{
		return;
	}

	// find corrections

	glm::vec3 derivatives[4] = { glm::vec3(0.0f, 0.0f, 0.0f) };

	derivatives[1] = -(glm::cross(vertices_test_coords_copy[2], triangle_b_normal) + glm::cross(triangle_a_normal, vertices_test_coords_copy[2]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[2]));
	derivatives[1] -= (glm::cross(vertices_test_coords_copy[3], triangle_a_normal) + glm::cross(triangle_b_normal, vertices_test_coords_copy[3]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[3]));

	derivatives[2] = (glm::cross(vertices_test_coords_copy[1], triangle_b_normal) + glm::cross(triangle_a_normal, vertices_test_coords_copy[1]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[2]));

	derivatives[3] = (glm::cross(vertices_test_coords_copy[1], triangle_a_normal) + glm::cross(triangle_b_normal, vertices_test_coords_copy[1]) * current_angle_cos) /
		glm::length(glm::cross(vertices_test_coords_copy[1], vertices_test_coords_copy[3]));

	derivatives[0] = -derivatives[1] - derivatives[2] - derivatives[3];

	const float coefficient = 1.0f / sqrtf(1.0f - current_angle_cos * current_angle_cos);
	derivatives[0] *= coefficient;
	derivatives[1] *= coefficient;
	derivatives[2] *= coefficient;
	derivatives[3] *= coefficient;

	// norms of derivatives
	glm::vec4 derivatives_norms =
	{
		glm::dot(derivatives[0],derivatives[0]),
		glm::dot(derivatives[1],derivatives[1]),
		glm::dot(derivatives[2],derivatives[2]),
		glm::dot(derivatives[3],derivatives[3])
	};

	glm::vec4 opposite_masses =
	{
		cloth.getOppositeMass(host_vertices[constraint_vertices[0]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[1]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[2]]),
		cloth.getOppositeMass(host_vertices[constraint_vertices[3]]) };

	const float delta_lambda = (-C - overline_alpha * lambda_accumulator) / (glm::dot(opposite_masses, derivatives_norms) + overline_alpha);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::FIXED_ANGLE]] =
	{
		delta_lambda * opposite_masses[0] * derivatives[0],
		delta_lambda * opposite_masses[1] * derivatives[1],
		delta_lambda * opposite_masses[2] * derivatives[2],
		delta_lambda * opposite_masses[3] * derivatives[3]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::FIXED_ANGLE]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1]),
		glm::length(deltas[2]),
		glm::length(deltas[3])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH || deltas_lengths[2] > MAX_LENGTH || deltas_lengths[3] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1] + deltas_lengths[2] + deltas_lengths[3];
	}
#endif

	// apply corrections
	// now we need pointers but not copies
	cloth.getTestCoords(constraint_vertices[0]) += delta_lambda * opposite_masses[0] * derivatives[0];
	cloth.getTestCoords(constraint_vertices[1]) += delta_lambda * opposite_masses[1] * derivatives[1];
	cloth.getTestCoords(constraint_vertices[2]) += delta_lambda * opposite_masses[2] * derivatives[2];
	cloth.getTestCoords(constraint_vertices[3]) += delta_lambda * opposite_masses[3] * derivatives[3];

	// update lambda for XPBD
	lambda_accumulator += delta_lambda;
}

void EvaluateConstraintsMultithread::evaluateSelfVertexTriangleCollision(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id)
{
	// you can read about this constraint in
	// "Cloth Self Collision with Predictive Contacts", p.3

	// get constraint data
	// SELF_VERTEX_TRIANGLE_COLLISION: 1 - vertex, 3 - triangle
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION];

	// all vertices here are real
	uint32_t constraint_vertices[vertices_count] = { 0u };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = host_vertices[raw_constraint_vertices[i]];
	}

	glm::vec3* vertex_test_coords = &cloth.getTestCoords(constraint_vertices[0]);
	glm::vec3* triangle_vertices_test_coords[3] =
	{ &cloth.getTestCoords(constraint_vertices[1]), &cloth.getTestCoords(constraint_vertices[2]), &cloth.getTestCoords(constraint_vertices[3]) };

	// SELF_VERTEX_TRIANGLE_COLLISION: 1 - optimal distance, 1 - flag if vertex is above the triangle
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data
	const float optimal_distance = constraint_data[0];

	glm::vec3 normal = MathUtils::triangleNormal(*triangle_vertices_test_coords[0], *triangle_vertices_test_coords[1], *triangle_vertices_test_coords[2]);
	const float opposite_normal_length = 1.0f / glm::length(normal);
	normal *= opposite_normal_length;

	const bool vertex_is_above_triangle = MathUtils::floatToBool(constraint_data[1]);

	normal = vertex_is_above_triangle ? normal : -normal;

	const glm::vec3 vector_0_vertex = *vertex_test_coords - *triangle_vertices_test_coords[0];
	const float C = glm::dot(normal, vector_0_vertex) - optimal_distance;

	// we don't need to perform corrections
	if (C > 0.0f)
	{
		return;
	}

	// find corrections
	const float x2 = normal.x * normal.x;
	const float y2 = normal.y * normal.y;
	const float z2 = normal.z * normal.z;

	glm::vec3 N_n =
	{
		normal.x - (x2 * normal.x) - (normal.x * y2) - (normal.x * z2),
		(-x2 * normal.y) + normal.y - (y2 * normal.y) - (normal.y * z2),
		(-x2 * normal.z) - (y2 * normal.z) + normal.z - (z2 * normal.z)
	};

	N_n *= opposite_normal_length;

	const glm::vec3 derivatives[4] =
	{
		normal,
		glm::cross(*triangle_vertices_test_coords[1] - *triangle_vertices_test_coords[2], N_n) - normal,
		glm::cross(*triangle_vertices_test_coords[2] - *triangle_vertices_test_coords[0], N_n) - normal,
		glm::cross(*triangle_vertices_test_coords[1] - *triangle_vertices_test_coords[0], N_n) - normal
	};

	const glm::vec4 derivatives_norms =
	{
		1.0f,
		glm::dot(derivatives[1],derivatives[1]),
		glm::dot(derivatives[2],derivatives[2]),
		glm::dot(derivatives[3],derivatives[3])
	};

	const glm::vec4 opposite_masses =
	{
		cloth.getOppositeMass(constraint_vertices[0]),
		cloth.getOppositeMass(constraint_vertices[1]),
		cloth.getOppositeMass(constraint_vertices[2]),
		cloth.getOppositeMass(constraint_vertices[3]) };

	const float lambda = -C / glm::dot(opposite_masses, derivatives_norms);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION]] =
	{
		lambda * opposite_masses[0] * derivatives[0],
		lambda * opposite_masses[1] * derivatives[1],
		lambda * opposite_masses[2] * derivatives[2]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1]),
		glm::length(deltas[2])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH || deltas_lengths[2] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1] + deltas_lengths[2];
	}
#endif

	// apply corrections
	* vertex_test_coords += lambda * opposite_masses[0] * derivatives[0];
	*triangle_vertices_test_coords[0] += lambda * opposite_masses[1] * derivatives[1];
	*triangle_vertices_test_coords[1] += lambda * opposite_masses[2] * derivatives[2];
	*triangle_vertices_test_coords[2] += lambda * opposite_masses[3] * derivatives[3];
}

void EvaluateConstraintsMultithread::evaluateSelfVertexTriangleCollisionBetweenLayers(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id)
{
	// you can read about this constraint in
	// "Cloth Self Collision with Predictive Contacts", p.3-4

	// get constraint data
	// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: 1 - vertex, 3 - triangle
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS];

	// all vertices here are real
	uint32_t constraint_vertices[vertices_count] = { 0u };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = host_vertices[raw_constraint_vertices[i]];
	}

	glm::vec3* vertex_test_coords = &cloth.getTestCoords(constraint_vertices[0]);
	glm::vec3* triangle_vertices_test_coords[3] = { &cloth.getTestCoords(constraint_vertices[1]), &cloth.getTestCoords(constraint_vertices[2]), &cloth.getTestCoords(constraint_vertices[3]) };

	// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: 1 - optimal distance
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get tata special for this constraint
	const int vertex_layer = cloth.getVertexPartId(constraint_vertices[0]);
	const int triangle_layer = cloth.getVertexPartId(constraint_vertices[1]);

	// get special data
	const float optimal_distance = constraint_data[0];

	glm::vec3 normal = MathUtils::triangleNormal(*triangle_vertices_test_coords[0], *triangle_vertices_test_coords[1], *triangle_vertices_test_coords[2]);
	const float opposite_normal_length = 1.0f / glm::length(normal);
	normal *= opposite_normal_length;
	normal = (vertex_layer > triangle_layer) ? normal : -normal;

	const glm::vec3 vector_0_vertex = *vertex_test_coords - *triangle_vertices_test_coords[0];
	const float C = glm::dot(normal, vector_0_vertex) - optimal_distance;

	// we don't need to perform corrections
	if (C > 0.0f)
	{
		return;
	}

	// find vertices corrections
	const float x2 = normal.x * normal.x;
	const float y2 = normal.y * normal.y;
	const float z2 = normal.z * normal.z;

	glm::vec3 N_n =
	{
		normal.x - (x2 * normal.x) - (normal.x * y2) - (normal.x * z2),
		(-x2 * normal.y) + normal.y - (y2 * normal.y) - (normal.y * z2),
		(-x2 * normal.z) - (y2 * normal.z) + normal.z - (z2 * normal.z)
	};

	N_n *= opposite_normal_length;

	const glm::vec3 derivatives[4] =
	{
		normal,
		glm::cross(*triangle_vertices_test_coords[1] - *triangle_vertices_test_coords[2], N_n) - normal,
		glm::cross(*triangle_vertices_test_coords[2] - *triangle_vertices_test_coords[0], N_n) - normal,
		glm::cross(*triangle_vertices_test_coords[1] - *triangle_vertices_test_coords[0], N_n) - normal
	};

	const glm::vec4 derivatives_norms =
	{
		1.0f,
		glm::dot(derivatives[1],derivatives[1]),
		glm::dot(derivatives[2],derivatives[2]),
		glm::dot(derivatives[3],derivatives[3])
	};

	const glm::vec4 opposite_masses =
	{
		cloth.getOppositeMass(constraint_vertices[0]),
		cloth.getOppositeMass(constraint_vertices[1]),
		cloth.getOppositeMass(constraint_vertices[2]),
		cloth.getOppositeMass(constraint_vertices[3]) };

	const float lambda = -C / glm::dot(opposite_masses, derivatives_norms);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS]] =
	{
		lambda * opposite_masses[0] * derivatives[0],
		lambda * opposite_masses[1] * derivatives[1],
		lambda * opposite_masses[2] * derivatives[2]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1]),
		glm::length(deltas[2])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH || deltas_lengths[2] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1] + deltas_lengths[2];
	}
#endif

	// apply corrections
	* vertex_test_coords += lambda * opposite_masses[0] * derivatives[0];
	*triangle_vertices_test_coords[0] += lambda * opposite_masses[1] * derivatives[1];
	*triangle_vertices_test_coords[1] += lambda * opposite_masses[2] * derivatives[2];
	*triangle_vertices_test_coords[2] += lambda * opposite_masses[3] * derivatives[3];
}

void EvaluateConstraintsMultithread::evaluateSelfEdgeEdgeCollision(Cloth& cloth, ConstraintsBuffers& buffers, int constraint_id)
{
	// you can read about this constraint in
	// "Cloth Self Collision with Predictive Contacts", p.3

	// get constraint data
	// : 2 - edge a, 2 - edge b
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION];

	// all vertices here are real
	uint32_t constraint_vertices[vertices_count] = { 0u };
	const glm::vec3* vertices_coords[vertices_count] = { nullptr };
	glm::vec3* vertices_test_coords[vertices_count] = { nullptr };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = host_vertices[raw_constraint_vertices[i]];
		vertices_coords[i] = &cloth.getCoords(constraint_vertices[i]);
		vertices_test_coords[i] = &cloth.getTestCoords(constraint_vertices[i]);
	}

	// SELF_EDGE_EDGE_COLLISION: 1 - optimal distance, 2 - parameters of vertices of the edges
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get specific data
	const float optimal_distance = constraint_data[0];
	const float edge_1_parameter = constraint_data[1];
	const float edge_2_parameter = constraint_data[2];

	const glm::vec3 point_on_edge_1 = glm::mix(*vertices_test_coords[0], *vertices_test_coords[1], edge_1_parameter);
	const glm::vec3 point_on_edge_2 = glm::mix(*vertices_test_coords[2], *vertices_test_coords[3], edge_2_parameter);
	const glm::vec3 vector_from_edge_1_to_edge_2_now = point_on_edge_2 - point_on_edge_1;
	const glm::vec3 point_on_edge_1_before = glm::mix(*vertices_coords[0], *vertices_coords[1], edge_1_parameter);
	const glm::vec3 point_on_edge_2_before = glm::mix(*vertices_coords[2], *vertices_coords[3], edge_2_parameter);
	const glm::vec3 vector_from_edge_1_to_edge_2_before = glm::normalize(point_on_edge_2_before - point_on_edge_1_before);
	const float C = glm::dot(vector_from_edge_1_to_edge_2_before, vector_from_edge_1_to_edge_2_now) - optimal_distance;

	// we don't need to perform corrections
	if (C > 0.0f)
	{
		return;
	}

	// find corrections
	glm::vec3 derivatives[4]{
		(edge_1_parameter - 1.0f) * vector_from_edge_1_to_edge_2_before,
		-edge_1_parameter * vector_from_edge_1_to_edge_2_before,
		(1.0f - edge_2_parameter) * vector_from_edge_1_to_edge_2_before,
		edge_2_parameter * vector_from_edge_1_to_edge_2_before };

	const glm::vec4 derivatives_norms =
	{
		glm::dot(derivatives[0],derivatives[0]),
		glm::dot(derivatives[1],derivatives[1]),
		glm::dot(derivatives[2],derivatives[2]),
		glm::dot(derivatives[3],derivatives[3])
	};

	const glm::vec4 opposite_masses =
	{
		cloth.getOppositeMass(constraint_vertices[0]),
		cloth.getOppositeMass(constraint_vertices[1]),
		cloth.getOppositeMass(constraint_vertices[2]),
		cloth.getOppositeMass(constraint_vertices[3]) };

	const float lambda = -C / glm::dot(opposite_masses, derivatives_norms);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION]] =
	{
		lambda * opposite_masses[0] * derivatives[0],
		lambda * opposite_masses[1] * derivatives[1],
		lambda * opposite_masses[2] * derivatives[2],
		lambda * opposite_masses[3] * derivatives[3]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1]),
		glm::length(deltas[2]),
		glm::length(deltas[3])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH || deltas_lengths[2] > MAX_LENGTH || deltas_lengths[3] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1] + deltas_lengths[2] + deltas_lengths[3];
	}
#endif

	// apply corrections
	* vertices_test_coords[0] += opposite_masses[0] * lambda * derivatives[0];
	*vertices_test_coords[1] += opposite_masses[1] * lambda * derivatives[1];
	*vertices_test_coords[2] += opposite_masses[2] * lambda * derivatives[2];
	*vertices_test_coords[3] += opposite_masses[3] * lambda * derivatives[3];
}

void EvaluateConstraintsMultithread::evaluateColliderVertexTriangleCollision(Cloth& cloth, const Collider& colliders, ConstraintsBuffers& buffers, int constraint_id)
{
	// this constraint acts same as in case of self collision
	// you can read about this constraint in
	// "Cloth Self Collision with Predictive Contacts", p.3

	// get constraint data
	// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - one vertex of the cloth
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION];

	// all vertices here are real
	uint32_t constraint_vertices[vertices_count] = { 0u };
	glm::vec3* vertices_test_coords[vertices_count] = { nullptr };
	constraint_vertices[0] = host_vertices[raw_constraint_vertices[0]];
	vertices_test_coords[0] = &cloth.getTestCoords(constraint_vertices[0]);


	// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - recommended distance,  1 - collider triangle id
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get special data
	const float optimal_distance = constraint_data[0];
	const int collider_triangle_id = (int)roundf(constraint_data[1]);

	const std::array<const glm::vec3*, 3> collider_triangle_coords = colliders.getTriangleCoords(collider_triangle_id);

	glm::vec3 triangle_normal = MathUtils::triangleNormal(*collider_triangle_coords[0], *collider_triangle_coords[1], *collider_triangle_coords[2]);
	const glm::vec3 vector_0_vertex = *vertices_test_coords[0] - *collider_triangle_coords[0];
	const float C = glm::dot(triangle_normal, vector_0_vertex) - optimal_distance;

	// we don't need to perform corrections
	if (C > 0.0f)
	{
		return;
	}

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION]] =
	{
		-C * triangle_normal
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION]] =
	{
		glm::length(deltas[0])
	};

	if (deltas_lengths[0] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0];
	}
#endif

	// apply corrections
	// derivative is triangle_normal
	// lambda is -C
	* vertices_test_coords[0] += -C * triangle_normal;
}

void EvaluateConstraintsMultithread::evaluateColliderEdgeEdgeCollision(Cloth& cloth, const Collider& colliders, ConstraintsBuffers& buffers, int constraint_id)
{
	// this constraint acts same as in case of self collision
	// you can read about this constraint in
	// "Cloth Self Collision with Predictive Contacts", p.3

	// get constraint data
	// COLLIDER_EDGE_EDGE_COLLISION: 2 - one edge
	const uint32_t* raw_constraint_vertices = buffers.getConstraintVertices(constraint_id);
	const std::vector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
	constexpr int vertices_count = CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION];

	// all vertices here are real
	uint32_t constraint_vertices[vertices_count] = { 0u };
	const glm::vec3* vertices_coords[vertices_count] = { nullptr };
	glm::vec3* vertices_test_coords[vertices_count] = { nullptr };
	for (int i = 0; i < vertices_count; ++i)
	{
		constraint_vertices[i] = host_vertices[raw_constraint_vertices[i]];
		vertices_coords[i] = &cloth.getCoords(constraint_vertices[i]);
		vertices_test_coords[i] = &cloth.getTestCoords(constraint_vertices[i]);
	}

	// COLLIDER_EDGE_EDGE_COLLISION: 1 - optimal distance, 1 - point parameter on cloth edge, 1 - point parameter on collider edge,
	// 1 - collider edge vertex a, 1 - collider edge vertex b
	float* constraint_data = buffers.getConstraintData(constraint_id);

	// get specific data
	const float optimal_distance = constraint_data[0];
	const float cloth_edge_parameter = constraint_data[1];
	const float object_edge_parameter = constraint_data[2];
	const int collider_vertex_a_id = (int)roundf(constraint_data[3]);
	const int collider_vertex_b_id = (int)roundf(constraint_data[4]);

	const glm::vec3 point_on_object_edge =
		glm::mix(colliders.getCoords(collider_vertex_a_id), colliders.getCoords(collider_vertex_b_id), object_edge_parameter);
	const glm::vec3 point_on_cloth_edge_before = glm::mix(*vertices_coords[0], *vertices_coords[1], cloth_edge_parameter);
	const glm::vec3 point_on_cloth_edge = glm::mix(*vertices_test_coords[0], *vertices_test_coords[1], cloth_edge_parameter);
	const glm::vec3 vector_from_cloth_edge_to_object_edge_before = glm::normalize(point_on_object_edge - point_on_cloth_edge_before);
	const glm::vec3 vector_from_cloth_edge_to_object_edge_now = point_on_object_edge - point_on_cloth_edge;

	const float C = glm::dot(vector_from_cloth_edge_to_object_edge_before, vector_from_cloth_edge_to_object_edge_now) - optimal_distance;

	// we don't need to perform corrections
	if (C > 0.0f)
	{
		return;
	}

	// find corrections
	const glm::vec3 derivatives[2] =
	{
		((cloth_edge_parameter - 1.0f)) * vector_from_cloth_edge_to_object_edge_before,
		-cloth_edge_parameter * vector_from_cloth_edge_to_object_edge_before
	};

	const glm::vec2 opposite_masses = { cloth.getOppositeMass(constraint_vertices[0]), cloth.getOppositeMass(constraint_vertices[1]) };
	const glm::vec2 derivatives_norms = { glm::dot(derivatives[0],derivatives[0]), glm::dot(derivatives[1],derivatives[1]) };
	const float lambda = -C / glm::dot(opposite_masses, derivatives_norms);

#ifdef CHECK_DELTAS
	glm::vec3 deltas[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION]] =
	{
		lambda * cloth.getOppositeMass(constraint_vertices[0]) * derivatives[0],
		lambda * cloth.getOppositeMass(constraint_vertices[1]) * derivatives[1]
	};

	float deltas_lengths[CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION]] =
	{
		glm::length(deltas[0]),
		glm::length(deltas[1])
	};

	if (deltas_lengths[0] > MAX_LENGTH || deltas_lengths[1] > MAX_LENGTH)
	{
		const float total_length = deltas_lengths[0] + deltas_lengths[1];
	}
#endif

	// apply corrections
	* vertices_test_coords[0] += lambda * cloth.getOppositeMass(constraint_vertices[0]) * derivatives[0];
	*vertices_test_coords[1] += lambda * cloth.getOppositeMass(constraint_vertices[1]) * derivatives[1];
}