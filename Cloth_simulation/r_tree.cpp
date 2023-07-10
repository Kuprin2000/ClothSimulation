// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "r_tree.h"
#include <stack>

void RTree::insertVertices(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, int first_vertex_index, ObjectType object_type, float gap)
{
	for (auto iter = vertices_coords.cbegin(); iter != vertices_coords.end(); ++iter)
	{
		m_current_node.m_object_type = object_type;
		m_current_node.m_primitive_id = first_vertex_index;
		m_current_node.m_primitive_type = PrimitiveType::VERTEX;
		m_current_node.m_children = {};
		m_current_node.m_parent = NO_PARENT;
		m_current_node.m_bnd_box.setVertex(*iter, gap);

		insertNodeInternal();

		++first_vertex_index;
	}
}

void RTree::insertMovingVertices(const AlignedVector::AlignedVector<glm::vec3>& old_vertices_coords, const AlignedVector::AlignedVector<glm::vec3>& new_vertices_coords,
	int first_vertex_index, ObjectType object_type, float gap)
{
	for (int i = 0; i < new_vertices_coords.size(); ++i)
	{
		m_current_node.m_object_type = object_type;
		m_current_node.m_primitive_id = first_vertex_index;
		m_current_node.m_primitive_type = PrimitiveType::VERTEX;
		m_current_node.m_children = {};
		m_current_node.m_parent = NO_PARENT;
		m_current_node.m_bnd_box.setMovingVertex(old_vertices_coords[i], new_vertices_coords[i], gap);

		insertNodeInternal();

		++first_vertex_index;
	}
}

void RTree::insertTriangles(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices,
	int first_triangle_index, ObjectType object_type, float gap)
{
	glm::vec3 triangle_vertices_coords[3] = { glm::vec3() };

	for (int i = 0; i < vertices_indices.size(); ++i)
	{
		triangle_vertices_coords[0] = vertices_coords[vertices_indices[i][0]];
		triangle_vertices_coords[1] = vertices_coords[vertices_indices[i][1]];
		triangle_vertices_coords[2] = vertices_coords[vertices_indices[i][2]];

		m_current_node.m_object_type = object_type;
		m_current_node.m_primitive_id = first_triangle_index;
		m_current_node.m_primitive_type = PrimitiveType::TRIANGLE;
		m_current_node.m_children = {};
		m_current_node.m_parent = NO_PARENT;
		m_current_node.m_bnd_box.setTriangle(triangle_vertices_coords, gap);

		insertNodeInternal();

		++first_triangle_index;
	}
}

void RTree::insertMovingTriangles(const AlignedVector::AlignedVector<glm::vec3>& old_vertices_coords, const AlignedVector::AlignedVector<glm::vec3>& new_vertices_coords,
	const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, int first_triangle_index, ObjectType object_type, float gap)
{
	glm::vec3 triangle_vertices_coords[3] = { glm::vec3() };
	glm::vec3 triangle_vertices_new_coords[3] = { glm::vec3() };

	for (int i = 0; i < vertices_indices.size(); ++i)
	{
		triangle_vertices_coords[0] = old_vertices_coords[vertices_indices[i][0]];
		triangle_vertices_coords[1] = old_vertices_coords[vertices_indices[i][1]];
		triangle_vertices_coords[2] = old_vertices_coords[vertices_indices[i][2]];

		triangle_vertices_new_coords[0] = new_vertices_coords[vertices_indices[i][0]];
		triangle_vertices_new_coords[1] = new_vertices_coords[vertices_indices[i][1]];
		triangle_vertices_new_coords[2] = new_vertices_coords[vertices_indices[i][2]];

		m_current_node.m_object_type = object_type;
		m_current_node.m_primitive_id = first_triangle_index;
		m_current_node.m_primitive_type = PrimitiveType::TRIANGLE;
		m_current_node.m_children = {};
		m_current_node.m_parent = NO_PARENT;
		m_current_node.m_bnd_box.setMovingTriangle(triangle_vertices_coords, triangle_vertices_new_coords, gap);

		insertNodeInternal();

		++first_triangle_index;
	}
}

std::vector<int> RTree::findCollisionsWithBndBox(const BoundingBox& bnd_box)
{
	int current_node = ROOT_INDEX;

	std::stack<int> candidates_stack;
	candidates_stack.push(ROOT_INDEX);

	std::vector<int> result;

	while (!candidates_stack.empty())
	{
		current_node = candidates_stack.top();
		candidates_stack.pop();

		const std::vector<int>& current_node_children = m_nodes.getChildren(current_node);

		if (m_nodes.nodeIsLeaf(current_node))
		{
			for (int i = 0; i < current_node_children.size(); ++i)
			{
				if (BoundingBox::thereIsOVerlap(m_nodes.getBndBox(current_node_children[i]), bnd_box))
				{
					result.push_back(current_node_children[i]);
				}
			}
		}

		else
		{
			for (int i = 0; i < current_node_children.size(); ++i)
			{
				if (BoundingBox::thereIsOVerlap(m_nodes.getBndBox(current_node_children[i]), bnd_box))
				{
					candidates_stack.push(current_node_children[i]);
				}
			}
		}
	}

	return result;
}

void RTree::insertNodeInternal()
{
	const int new_node_id = m_nodes.getSize();

	m_nodes.pushToNodes(m_current_node.m_object_type, m_current_node.m_primitive_id, m_current_node.m_primitive_type,
		m_current_node.m_children, m_current_node.m_parent, m_current_node.m_bnd_box);

	if (m_current_node.m_object_type == ObjectType::CLOTH)
	{
		m_cloth_primitives_nodes.push_back(new_node_id);
	}

	const int node_to_push_into_id = chooseSubtree(new_node_id);

	if (m_nodes.getChildren(node_to_push_into_id).size() < m_max_node_children_count)
	{
		m_nodes.setParent(node_to_push_into_id, new_node_id);
		m_nodes.getChildren(node_to_push_into_id).push_back(new_node_id);

		for (int node_id = node_to_push_into_id; node_id != NO_PARENT; node_id = m_nodes.getParent(node_id))
		{
			m_nodes.nodeRecalcBndBox(node_id);
		}
	}

	else
	{
		splitNode(node_to_push_into_id, new_node_id);
	}
}

int RTree::chooseSubtree(int new_node_id)
{
	int current_node_id = ROOT_INDEX;

	while (true)
	{
		if (m_nodes.nodeIsLeaf(current_node_id) ||
			(current_node_id == ROOT_INDEX && m_nodes.getChildren(current_node_id).empty()))
		{
			return current_node_id;
		}

		current_node_id = m_nodes.getChildren(current_node_id)[findOptimalChild(current_node_id, new_node_id)];
	}
}

int RTree::findOptimalChild(int parent_node, int new_node)
{
	// nodes we will analyze
	const std::vector<int>& nodes = m_nodes.getChildren(parent_node);

	// bounding box of a node
	BoundingBox node_bnd_box;

	// data structures to analyze volumes
	float volume_before_addition = 0.0f;
	float volume_after_addition = 0.0f;
	std::vector<float> volumes_after_addition(nodes.size(), 0.0f);
	std::vector<float> volume_deltas(nodes.size(), 0.0f);

	// try to insert point into every node
	// i is a node we try to add a point
	for (int i = 0; i < nodes.size(); ++i)
	{
		node_bnd_box = m_nodes.getBndBox(nodes[i]);

		volume_before_addition = node_bnd_box.getVolume();
		node_bnd_box.addBndBox(m_nodes.getBndBox(new_node));

		// calc new volume and volume delta
		volume_after_addition = node_bnd_box.getVolume();
		volumes_after_addition[i] = volume_after_addition;
		volume_deltas[i] = volume_after_addition - volume_before_addition;
	}

	// firstly we look for nodes with the least volume delta
	float min_volume_delta = FLT_MAX;
	std::vector<int> candidates;

	for (int i = 0; i < nodes.size(); ++i)
	{
		float tmp_value = volume_deltas[i] - min_volume_delta;
		if (tmp_value < -FLT_EPSILON)
		{
			min_volume_delta = volume_deltas[i];
			candidates.clear();
			candidates.push_back(i);
		}
		else if (fabsf(tmp_value) < FLT_EPSILON)
		{
			candidates.push_back(i);
		}
	}

	// if there is only one candidate, return it's index
	if (candidates.size() == 1)
	{
		return candidates[0];
	}

	// and return candidate with the least volume
	float min_volume = FLT_MAX;
	int min_volume_index = 0;
	for (int i = 0; i < candidates.size(); ++i)
	{
		if (min_volume > volumes_after_addition[candidates[i]])
		{
			min_volume = volumes_after_addition[candidates[i]];
			min_volume_index = i;
		}
	}

	return min_volume_index;
}

void RTree::splitNode(int node_to_split_id, int node_to_insert_id)
{
	// both nodes node_to_split and node_to_insert are in m_nodes vector already

	// nodes we will split
	std::vector<int> free_nodes = m_nodes.getChildren(node_to_split_id);
	free_nodes.push_back(node_to_insert_id);

	std::vector<int> groups[2];
	float groups_volumes[2] = { 0.0f };
	int initial_nodes[2] = { 0 };
	int must_add_to_groups[2] = { 0 };
	int groups_sizes[2] = { 0 };

	int chosen_group = 0;
	int chosen_node = 0;
	float chosen_group_new_volume = 0.0f;
	BoundingBox chosen_group_new_bnd_box;

	bool need_to_recalc_bnd_box = false;

	pickSeeds(free_nodes, initial_nodes, groups_volumes);

	groups[0].push_back(free_nodes[initial_nodes[0]]);
	groups[1].push_back(free_nodes[initial_nodes[1]]);

	BoundingBox groups_bnd_boxes[2] = { m_nodes.getBndBox(groups[0][0]), m_nodes.getBndBox(groups[1][0]) };

	free_nodes.erase(free_nodes.begin() + initial_nodes[initial_nodes[0] < initial_nodes[1]]);
	free_nodes.erase(free_nodes.begin() + initial_nodes[!(initial_nodes[0] < initial_nodes[1])]);

	while (!free_nodes.empty())
	{
		must_add_to_groups[0] = std::max(m_min_node_children_count - (int)groups[0].size(), 0);
		must_add_to_groups[1] = std::max(m_min_node_children_count - (int)groups[1].size(), 0);

		if (must_add_to_groups[0] + must_add_to_groups[1] == (long)free_nodes.size())
		{
			groups[0].insert(groups[0].end(), free_nodes.begin(), free_nodes.begin() + must_add_to_groups[0]);
			groups[1].insert(groups[1].end(), free_nodes.begin() + must_add_to_groups[0], free_nodes.end());

			free_nodes.clear();
			need_to_recalc_bnd_box = true;
			continue;
		}

		groups_sizes[0] = (int)groups[0].size();
		groups_sizes[1] = (int)groups[1].size();
		findNext(free_nodes, groups_bnd_boxes, groups_volumes, groups_sizes,
			chosen_group, chosen_node, chosen_group_new_volume, chosen_group_new_bnd_box);

		groups[chosen_group].push_back(free_nodes[chosen_node]);
		groups_bnd_boxes[chosen_group] = chosen_group_new_bnd_box;
		groups_volumes[chosen_group] = chosen_group_new_volume;

		free_nodes.erase(free_nodes.begin() + chosen_node);
	}

	// create two new nodes. We push second node to the end of m_nodes vector but first_new_node will be inserted later
	// so we edit second node with m_nodes vector but first_new_node should be edited directly
	int first_new_node_id = node_to_split_id;
	const int second_new_node_id = m_nodes.getSize();

	m_current_node.m_object_type = ObjectType::NONE;
	m_current_node.m_primitive_id = NO_PRIMITIVE;
	m_current_node.m_primitive_type = PrimitiveType::NONE;
	m_current_node.m_children = groups[0];
	m_current_node.m_parent = NO_PARENT;
	m_current_node.m_bnd_box = groups_bnd_boxes[0];

	if (need_to_recalc_bnd_box)
	{
		m_current_node.m_bnd_box.setVoid();

		for (const auto child : m_current_node.m_children)
		{
			m_current_node.m_bnd_box.addBndBox(m_nodes.getBndBox(child));
		}
	}

	m_nodes.pushToNodes(ObjectType::NONE, NO_PRIMITIVE, PrimitiveType::NONE, groups[1], NO_PARENT, BoundingBox());

	for (int i = 0; i < m_nodes.getChildren(second_new_node_id).size(); ++i)
	{
		m_nodes.setParent(second_new_node_id, m_nodes.getChildren(second_new_node_id)[i]);
	}

	if (need_to_recalc_bnd_box)
	{
		m_nodes.nodeRecalcBndBox(second_new_node_id);
	}
	else
	{
		m_nodes.getBndBox(second_new_node_id) = groups_bnd_boxes[1];
	}

	// if node_to_split is not a root
	if (node_to_split_id != ROOT_INDEX)
	{
		m_current_node.m_parent = m_nodes.getParent(node_to_split_id);
		m_nodes.setParent(m_nodes.getParent(node_to_split_id), second_new_node_id);

		for (int i = 0; i < m_current_node.m_children.size(); ++i)
		{
			m_nodes.setParent(first_new_node_id, m_current_node.m_children[i]);
		}

		// now first_new_node is in m_nodes too, so we should edit it with m_nodes vector
		m_nodes.setNode(m_current_node.m_object_type, m_current_node.m_primitive_id, m_current_node.m_primitive_type,
			m_current_node.m_children, m_current_node.m_parent, m_current_node.m_bnd_box, node_to_split_id);

		std::vector<int>& children_array_to_update = m_nodes.getChildren(m_nodes.getParent(node_to_split_id));
		for (int i = 0; i < children_array_to_update.size(); ++i)
		{
			if (children_array_to_update[i] == node_to_split_id)
			{
				children_array_to_update[i] = first_new_node_id;
			}
		}

		if (children_array_to_update.size() < m_max_node_children_count)
		{
			children_array_to_update.push_back(second_new_node_id);
		}
		else
		{
			for (int node_id = node_to_split_id; node_id != NO_PARENT; node_id = m_nodes.getParent(node_id))
			{
				m_nodes.nodeRecalcBndBox(node_id);
			}

			splitNode(m_nodes.getParent(node_to_split_id), second_new_node_id);
		}

		for (int node_id = second_new_node_id; node_id != NO_PARENT; node_id = m_nodes.getParent(node_id))
		{
			m_nodes.nodeRecalcBndBox(node_id);
		}
	}

	// if node_to_split is a root
	else
	{
		first_new_node_id = m_nodes.getSize();

		m_nodes.setNode(ObjectType::NONE, NO_PRIMITIVE, PrimitiveType::NONE, { first_new_node_id, second_new_node_id }, NO_PARENT, BoundingBox(), ROOT_INDEX);

		for (int i = 0; i < m_current_node.m_children.size(); ++i)
		{
			m_nodes.setParent(first_new_node_id, m_current_node.m_children[i]);
		}

		// now first_new_node is in m_nodes too, so we should edit it with m_nodes vector
		m_nodes.pushToNodes(m_current_node.m_object_type, m_current_node.m_primitive_id,
			m_current_node.m_primitive_type, m_current_node.m_children, m_current_node.m_parent, m_current_node.m_bnd_box);

		m_nodes.setParent(ROOT_INDEX, first_new_node_id);
		m_nodes.setParent(ROOT_INDEX, second_new_node_id);

		m_nodes.nodeRecalcBndBox(ROOT_INDEX);
	}
}

void RTree::pickSeeds(const std::vector<int>& nodes, int* optimal_nodes, float* optimal_nodes_volumes) const
{
	float node_1_volume = 0.0f;
	float node_2_volume = 0.0f;
	BoundingBox sum_of_bnd_boxes;

	float current_d = 0.0f;
	float min_d = FLT_MAX;

	for (int i = 0; i < nodes.size(); ++i)
	{
		for (int j = i + 1; j < nodes.size(); ++j)
		{
			node_1_volume = m_nodes.getBndBox(nodes[i]).getVolume();
			node_2_volume = m_nodes.getBndBox(nodes[j]).getVolume();
			sum_of_bnd_boxes = m_nodes.getBndBox(nodes[i]);
			sum_of_bnd_boxes.addBndBox(m_nodes.getBndBox(nodes[j]));

			current_d = sum_of_bnd_boxes.getVolume() - node_1_volume - node_2_volume;
			if (current_d < min_d)
			{
				min_d = current_d;
				optimal_nodes[0] = i;
				optimal_nodes[1] = j;
				optimal_nodes_volumes[0] = node_1_volume;
				optimal_nodes_volumes[1] = node_2_volume;
			}
		}
	}
}

void RTree::findNext(const std::vector<int>& free_nodes, const BoundingBox* groups_bnd_boxes,
	const float* groups_volumes_before_addition, const int* groups_elem_count, int& chosen_group,
	int& chosen_node, float& chosen_group_new_volume, BoundingBox& chosen_group_new_bnd_box) const
{
	int free_node;
	BoundingBox groups_bnd_boxes_after_addition[2];

	float groups_volumes_delta[2] = { 0.0f };

	float volume_delta = 0.0f;
	float max_volume_delta = -FLT_MAX;
	int optimal_node = 0;

	float optimal_groups_volumes_after_addition[2] = { 0.0f };
	float optimal_groups_volumes_delta[2] = { 0.0f };

	BoundingBox optimal_groups_bnd_boxes_after_addition[2];

	for (int i = 0; i < free_nodes.size(); ++i)
	{
		free_node = free_nodes[i];
		groups_bnd_boxes_after_addition[0] = groups_bnd_boxes[0];
		groups_bnd_boxes_after_addition[0].addBndBox(m_nodes.getBndBox(free_node));
		groups_bnd_boxes_after_addition[1] = groups_bnd_boxes[1];
		groups_bnd_boxes_after_addition[1].addBndBox(m_nodes.getBndBox(free_node));

		groups_volumes_delta[0] = groups_bnd_boxes_after_addition[0].getVolume() - groups_volumes_before_addition[0];
		groups_volumes_delta[1] = groups_bnd_boxes_after_addition[1].getVolume() - groups_volumes_before_addition[1];

		volume_delta = fabsf(groups_volumes_delta[0] - groups_volumes_delta[1]);

		if (volume_delta > max_volume_delta)
		{
			max_volume_delta = volume_delta;
			optimal_node = i;
			optimal_groups_volumes_after_addition[0] = groups_bnd_boxes_after_addition[0].getVolume();
			optimal_groups_volumes_after_addition[1] = groups_bnd_boxes_after_addition[1].getVolume();
			optimal_groups_volumes_delta[0] = groups_volumes_delta[0];
			optimal_groups_volumes_delta[1] = groups_volumes_delta[1];
			optimal_groups_bnd_boxes_after_addition[0] = groups_bnd_boxes_after_addition[0];
			optimal_groups_bnd_boxes_after_addition[1] = groups_bnd_boxes_after_addition[1];
		}
	}

	if (fabsf(optimal_groups_volumes_delta[1] - optimal_groups_volumes_delta[0]) > FLT_EPSILON)
	{
		chosen_group = int(optimal_groups_volumes_delta[1] - optimal_groups_volumes_delta[0] < -FLT_EPSILON);
	}

	else if (fabsf(groups_volumes_before_addition[0] - groups_volumes_before_addition[1]) > FLT_EPSILON)
	{
		chosen_group = groups_volumes_before_addition[0] - groups_volumes_before_addition[1] > FLT_EPSILON;
	}

	else if (groups_elem_count[0] != groups_elem_count[1])
	{
		chosen_group = (int)(groups_elem_count[0] > groups_elem_count[1]);
	}

	else
	{
		chosen_group = rand() % 2;
	}

	chosen_node = optimal_node;
	chosen_group_new_volume = optimal_groups_volumes_after_addition[chosen_group];
	chosen_group_new_bnd_box = optimal_groups_bnd_boxes_after_addition[chosen_group];
}
