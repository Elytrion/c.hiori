#pragma once

#include "aabb.h"
#include <stack>

namespace chiori
{
	static constexpr int null_node = -1;
	using QueryCallback = std::function<bool(unsigned int)>; //bool (*)(unsigned int);
	
	struct cTreeNode
	{
		AABB aabb;
		void* userData; // user data
		union
		{
			int parent;
			int next;
		};
		int child1;
		int child2;
		int height;
		
		bool IsLeaf() const
		{
			return (child1 == null_node);
		}
	};

	class cDynamicTree
	{
	public:
		cDynamicTree();
		~cDynamicTree();
		
		int InsertProxy(const AABB& inAABB, void* inUserData);
		int DestroyProxy(int inProxyID);
		bool MoveProxy(int inProxyID, const AABB& inAABB, const cVec2& inDisplacement);

		void* GetUserData(int inProxyID) const;
		
		const AABB& GetFattenedAABB(int inProxyID) const;

		void Query(const AABB& inAABB, QueryCallback callback) const;

		int GetHeight() const;
		int GetMaxBalance() const;
		float GetAreaRatio() const;
		// The shift formula is: position -= newOrigin
		void ShiftOrigin(const cVec2& newOrigin);

		// for testing
		void DisplayTree(std::function<void(int height, const AABB&)> drawFunc) const;
		
	private:
		int AllocateNode();
		void FreeNode(int node);

		void InsertLeaf(int node);
		void RemoveLeaf(int node);

		int Balance(int index);

		int ComputeHeight() const;
		int ComputeHeight(int nodeId) const;

		int m_root;
		cTreeNode* m_nodes;
		int m_nodeCount;
		int m_nodeCapacity;
		int m_freeList;
		/// This is used to incrementally traverse the tree for re-balancing.
		unsigned m_path;
		int m_insertionCount;
	};

	inline void* cDynamicTree::GetUserData(int proxyId) const
	{
		if (0 <= proxyId && proxyId < m_nodeCapacity)
		{
			return m_nodes[proxyId].userData;
		}
		throw std::out_of_range("Index out of range for tree");
	}

	inline const AABB& cDynamicTree::GetFattenedAABB(int proxyId) const
	{
		if (0 <= proxyId && proxyId < m_nodeCapacity)
		{
			return m_nodes[proxyId].aabb;
		}
		throw std::out_of_range("Index out of range for tree");
	}

	inline void cDynamicTree::Query(const AABB& inAABB, QueryCallback callback) const
	{
		std::stack<int> stack;
		stack.push(m_root);

		while (stack.size() > 0)
		{
			int nodeID = stack.top();
			stack.pop();
			if (nodeID == null_node)
				continue;

			const cTreeNode* node = m_nodes + nodeID;
			
			if (inAABB.intersects(node->aabb))
			{
				if (node->IsLeaf())
				{
					if (!callback(nodeID))
						return;
				}
				else
				{
					stack.push(node->child1);
					stack.push(node->child2);
				}
			}

		}
	}
}