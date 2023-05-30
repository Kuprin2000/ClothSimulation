#pragma once
#include <vector>
#include "indices_utils.h"
#include "bounding_box.h"

class RTree
{
public:
	static const int ROOT_INDEX = 0;
	static const int NO_PARENT = -1;
	static const uint32_t NO_PRIMITIVE = UINT32_MAX;

	enum class PrimitiveType : uint8_t
	{
		NONE,
		VERTEX,
		TRIANGLE
	};

	enum class ObjectType : uint8_t
	{
		NONE,
		CLOTH,
		COLLIDER
	};

	class RTreeNodes
	{
	public:
		_NODISCARD int getSize() const
		{
			return (int)m_object_type.size();
		}

		_NODISCARD ObjectType getObjectType(int index) const
		{
			return m_object_type[index];
		}

		_NODISCARD uint32_t getPrimitiveID(int index) const
		{
			return m_primitive_id[index];
		}

		_NODISCARD PrimitiveType getPrimitiveType(int index) const
		{
			return m_primitive_type[index];
		}

		_NODISCARD int getParent(int index) const
		{
			return m_parent[index];
		}

		void setObjectType(ObjectType new_obj_type, int index)
		{
			m_object_type[index] = new_obj_type;
		}

		void setPrimitiveID(uint32_t new_id, int index)
		{
			m_primitive_id[index] = new_id;
		}

		void setPrimitiveType(PrimitiveType new_type, int index)
		{
			m_primitive_type[index] = new_type;
		}

		void setParent(int new_parent, int index)
		{
			m_parent[index] = new_parent;
		}

		_NODISCARD std::vector<int>& getChildren(int index)
		{
			return m_children[index];
		}

		_NODISCARD BoundingBox& getBndBox(int index)
		{
			return m_bnd_box[index];
		}

		_NODISCARD const BoundingBox& getBndBox(int index) const
		{
			return m_bnd_box[index];
		}

		_NODISCARD bool nodeIsLeaf(int index)
		{
			if (!m_children[index].size())
			{
				return false;
			}

			return nodeIsObject(m_children[index][0]);
		}

		_NODISCARD bool nodeIsObject(int index)
		{
			return m_primitive_type[index] != PrimitiveType::NONE;
		}

		void nodeRecalcBndBox(int index)
		{
			BoundingBox& bnd_box = m_bnd_box[index];
			bnd_box.setVoid();

			for (const auto child : m_children[index])
			{
				bnd_box.addBndBox(m_bnd_box[child]);
			}
		}

		void pushToNodes(ObjectType object_type, uint32_t primitive_id, PrimitiveType primitive_type,
			const std::vector<int>& children, int parent, const BoundingBox& bnd_box)
		{
			m_object_type.push_back(object_type);
			m_primitive_id.push_back(primitive_id);
			m_primitive_type.push_back(primitive_type);
			m_children.push_back(children);
			m_parent.push_back(parent);
			m_bnd_box.push_back(bnd_box);
		}

		void setNode(ObjectType object_type, uint32_t primitive_id, PrimitiveType primitive_type,
			const std::vector<int>& children, int parent, const BoundingBox& bnd_box, int index)
		{
			m_object_type[index] = object_type;
			m_primitive_id[index] = primitive_id;
			m_primitive_type[index] = primitive_type;
			m_children[index] = children;
			m_parent[index] = parent;
			m_bnd_box[index] = bnd_box;
		}

		void clear()
		{
			m_object_type.clear();
			m_primitive_id.clear();
			m_primitive_type.clear();
			m_children.clear();
			m_parent.clear();
			m_bnd_box.clear();
		}

	private:
		std::vector<ObjectType> m_object_type;
		std::vector<uint32_t> m_primitive_id;
		std::vector<PrimitiveType> m_primitive_type;
		std::vector<std::vector<int>> m_children;
		std::vector<int> m_parent;
		std::vector<BoundingBox> m_bnd_box;
	};

	RTree() = delete;

	RTree(const RTree&) = delete;

	RTree(RTree&&) = default;

	RTree& operator=(RTree&& other) = default;

	RTree(int min_node_children_count, int max_node_children_count) : m_min_node_children_count(min_node_children_count),
		m_max_node_children_count(max_node_children_count)
	{
		m_nodes.pushToNodes(RTree::ObjectType::NONE, NO_PRIMITIVE, PrimitiveType::NONE, {}, NO_PARENT, BoundingBox());
	}

	void insertVertices(const std::vector<glm::vec3>& vertices_coords, int first_vertex_index, RTree::ObjectType object_type, float gap);

	void insertMovingVertices(const std::vector<glm::vec3>& old_vertices_coords, const std::vector<glm::vec3>& new_vertices_coords, int first_vertex_index, RTree::ObjectType object_type, float gap);

	void insertTriangles(const std::vector<glm::vec3>& vertices_coords, const std::vector<glm::uvec3>& vertices_indices,
		int first_triangle_index, RTree::ObjectType object_type, float gap);

	void insertMovingTriangles(const std::vector<glm::vec3>& old_vertices_coords, const std::vector<glm::vec3>& new_vertices_coords,
		const std::vector<glm::uvec3>& vertices_indices, int first_triangle_index, RTree::ObjectType object_type, float gap);

	_NODISCARD std::vector<int> findCollisionsWithBndBox(const BoundingBox& bnd_box);

	_NODISCARD const RTreeNodes& getNodes() const
	{
		return m_nodes;
	}

	_NODISCARD const std::vector<int>& getClothPrimitivesNodes() const
	{
		return m_cloth_primitives_nodes;
	}

	void clear()
	{
		m_nodes.clear();
		m_cloth_primitives_nodes.clear();

		m_nodes.pushToNodes(RTree::ObjectType::NONE, NO_PRIMITIVE, PrimitiveType::NONE, {}, NO_PARENT, BoundingBox());
	}

private:
	void insertNodeInternal();

	_NODISCARD int chooseSubtree(int new_node_id);

	_NODISCARD int findOptimalChild(int parent_node, int new_node);

	void splitNode(int node_to_split_id, int node_to_insert_id);

	void pickSeeds(const std::vector<int>& nodes, int* optimal_nodes, float* optimal_nodes_volumes) const;

	void findNext(const std::vector<int>& free_nodes, const BoundingBox* groups_bnd_boxes,
		const float* groups_volumes_before_addition, const int* groups_elem_count, int& chosen_group,
		int& chosen_node, float& chosen_group_new_volume, BoundingBox& chosen_group_new_bnd_box) const;

private:
	int m_min_node_children_count = 0;
	int m_max_node_children_count = 0;

	RTreeNodes m_nodes;

	struct
	{
		ObjectType m_object_type = ObjectType::NONE;
		uint32_t m_primitive_id = NO_PRIMITIVE;
		PrimitiveType m_primitive_type = PrimitiveType::NONE;
		std::vector<int> m_children;
		int m_parent = NO_PARENT;
		BoundingBox m_bnd_box;
	} m_current_node;

	std::vector<int> m_cloth_primitives_nodes;
};