#include "pch.h"
#include "aabbtree.h"

namespace chiori
{
	DynamicTree::DynamicTree()
	{
		m_root = null_node;
		
		m_nodeCapacity = commons::CTREE_START_CAPACITY;
		m_nodeCount = 0;
		m_nodes = new TreeNode[m_nodeCapacity]();

		// Set the free list (next points to the next free node)
		for (int i = 0; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_nodeCapacity - 1].next = null_node;
		m_nodes[m_nodeCapacity - 1].height = -1;
		
		m_freeList = 0;
		m_path = 0;
		m_insertionCount = 0;
	}

	DynamicTree::~DynamicTree()
	{
		// since the tree is flattened into an array,
		// this will clear the entire tree in one shot
		delete[] m_nodes;
	}

	int DynamicTree::AllocateNode()
	{
		if (m_freeList == null_node) // we need to expand node pool, free list is empty
		{
			cassert(m_nodeCount == m_nodeCapacity);

			TreeNode* oldNodes = m_nodes;
			m_nodeCapacity *= 2; // double capacity
			m_nodes = new TreeNode[m_nodeCapacity]();
			memcpy(m_nodes, oldNodes, m_nodeCount * sizeof(TreeNode)); // dangerous, make sure it doesnt break or leak!
			delete[] oldNodes;
			// Set the free list (next points to the next free node)
			for (int i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
			{
				m_nodes[i].next = i + 1;
				m_nodes[i].height = -1;
			}
			m_nodes[m_nodeCapacity - 1].next = null_node;
			m_nodes[m_nodeCapacity - 1].height = -1;
			m_freeList = m_nodeCount;
		}

		// Pull a node off the free list
		int nodeID = m_freeList;
		m_freeList = m_nodes[nodeID].next;
		m_nodes[nodeID].parent = null_node;
		m_nodes[nodeID].child1 = null_node;
		m_nodes[nodeID].child2 = null_node;
		m_nodes[nodeID].height = 0;
		m_nodes[nodeID].userData = nullptr;
		++m_nodeCount;
		return nodeID;
	}
	
	void DynamicTree::FreeNode(int nodeID)
	{
		cassert(0 <= nodeID && nodeID < m_nodeCapacity);
		cassert(0 < m_nodeCount);

		m_nodes[nodeID].next = m_freeList;
		m_nodes[nodeID].height = -1;
		m_freeList = nodeID;
		--m_nodeCount;
	}
	
	int DynamicTree::InsertProxy(const AABB& aabb, void* userData)
	{
		int pID = AllocateNode();
		
		vec2 fat{ commons::AABB_FATTEN_FACTOR, commons::AABB_FATTEN_FACTOR };
		m_nodes[pID].aabb.min = aabb.min - fat;
		m_nodes[pID].aabb.max = aabb.max + fat;
		m_nodes[pID].userData = userData;
		m_nodes[pID].height = 0;

		InsertLeaf(pID);

		return pID;
	}

	int DynamicTree::DestroyProxy(int proxyID)
	{
		cassert(0 <= proxyID && proxyID < m_nodeCapacity);
		cassert(m_nodes[proxyID].IsLeaf());
		RemoveLeaf(proxyID);
		FreeNode(proxyID);
		return proxyID;
	}

	bool DynamicTree::MoveProxy(int proxyID, const AABB& aabb, const vec2& disp)
	{
		cassert(0 <= proxyID && proxyID < m_nodeCapacity);

		cassert(m_nodes[proxyID].IsLeaf());
		
		if (m_nodes[proxyID].aabb.contains(aabb))
		{
			return false;
		}

		RemoveLeaf(proxyID);
		
		vec2 fat{ commons::AABB_FATTEN_FACTOR, commons::AABB_FATTEN_FACTOR };
		AABB b = aabb;
		b.min = b.min - fat;
		b.max = b.max + fat;

		vec2 pdisp = disp * 2.0f;

		if (pdisp.x < 0.0f)
		{
			b.min.x += pdisp.x;
		}
		else
		{
			b.max.x += pdisp.x;
		}
		
		if (pdisp.y < 0.0f)
		{
			b.min.y += pdisp.y;
		}
		else
		{
			b.max.y += pdisp.y;
		}

		m_nodes[proxyID].aabb = b;

		InsertLeaf(proxyID);
		return true;
	}

	void DynamicTree::InsertLeaf(int leaf)
	{
		++m_insertionCount;
		
		if (m_root == null_node)
		{
			m_root = leaf;
			m_nodes[m_root].parent = null_node;
			return;
		}
		// Find the best sibling for this node
		AABB leafAABB = m_nodes[leaf].aabb;
		int index = m_root;
		while (m_nodes[index].IsLeaf() == false)
		{
			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;

			float area = m_nodes[index].aabb.perimeter();
			
			AABB combinedAABB;
			combinedAABB.merge(m_nodes[index].aabb, leafAABB);
			float combinedArea = combinedAABB.perimeter();
			
			// Cost of creating a new parent for this node and the new leaf
			float cost = combinedArea * 2.0f;
			// Minimum cost of pushing the leaf further down the tree
			float inheritCost = 2.0f * (combinedArea - area);

			// Cost of descending into child1
			float costc1;
			if (m_nodes[child1].IsLeaf())
			{
				AABB aabb;
				aabb.merge(leafAABB, m_nodes[child1].aabb);
				costc1 = aabb.perimeter() + inheritCost;
			}
			else
			{
				AABB aabb;
				aabb.merge(leafAABB, m_nodes[child1].aabb);
				float oldArea = m_nodes[child1].aabb.perimeter();
				float newArea = aabb.perimeter();
				costc1 = (newArea - oldArea) + inheritCost;
			}
			
			// Cost of descending into child2
			float costc2;
			if (m_nodes[child2].IsLeaf())
			{
				AABB aabb;
				aabb.merge(leafAABB, m_nodes[child2].aabb);
				costc2 = aabb.perimeter() + inheritCost;
			}
			else
			{
				AABB aabb;
				aabb.merge(leafAABB, m_nodes[child2].aabb);
				float oldArea = m_nodes[child2].aabb.perimeter();
				float newArea = aabb.perimeter();
				costc2 = (newArea - oldArea) + inheritCost;
			}

			if (cost < costc1 && cost < costc2)
				break; // do not descend
			
			// descend to the one with lower cost
			index = (costc1 < costc2) ? child1 : child2;
		}
		
		int sibling = index;
		
		// Create a new parent.
		int oldParent = m_nodes[sibling].parent;
		int newParent = AllocateNode();
		m_nodes[newParent].parent = oldParent;
		m_nodes[newParent].userData = nullptr;
		m_nodes[newParent].aabb.merge(leafAABB, m_nodes[sibling].aabb);
		m_nodes[newParent].height = m_nodes[sibling].height + 1;

		if (oldParent != null_node)
		{
			// The sibling was not the root.
			if (m_nodes[oldParent].child1 == sibling)
			{
				m_nodes[oldParent].child1 = newParent;
			}
			else
			{
				m_nodes[oldParent].child2 = newParent;
			}

			m_nodes[newParent].child1 = sibling;
			m_nodes[newParent].child2 = leaf;
			m_nodes[sibling].parent = newParent;
			m_nodes[leaf].parent = newParent;
		}
		else
		{
			// The sibling was the root.
			m_nodes[newParent].child1 = sibling;
			m_nodes[newParent].child2 = leaf;
			m_nodes[sibling].parent = newParent;
			m_nodes[leaf].parent = newParent;
			m_root = newParent;
		}

		// Walk back up the tree fixing heights and AABBs
		index = m_nodes[leaf].parent;
		while (index != null_node)
		{
			index = Balance(index);

			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;
			cassert(child1 != null_node);
			cassert(child2 != null_node);
	
			m_nodes[index].height = 1 + max(m_nodes[child1].height, m_nodes[child2].height);
			m_nodes[index].aabb.merge(m_nodes[child1].aabb, m_nodes[child2].aabb);

			index = m_nodes[index].parent;
		}
	}

	void DynamicTree::RemoveLeaf(int leaf)
	{
		if (leaf == m_root)
		{
			m_root = null_node;
			return;
		}

		int parent = m_nodes[leaf].parent;
		int grandParent = m_nodes[parent].parent;
		int sibling;
		if (m_nodes[parent].child1 == leaf)
		{
			sibling = m_nodes[parent].child2;
		}
		else
		{
			sibling = m_nodes[parent].child1;
		}

		if (grandParent != null_node)
		{
			// Destroy parent and connect sibling to grandParent.
			if (m_nodes[grandParent].child1 == parent)
			{
				m_nodes[grandParent].child1 = sibling;
			}
			else
			{
				m_nodes[grandParent].child2 = sibling;
			}
			m_nodes[sibling].parent = grandParent;
			FreeNode(parent);

			// Adjust ancestor bounds.
			int index = grandParent;
			while (index != null_node)
			{
				index = Balance(index);

				int child1 = m_nodes[index].child1;
				int child2 = m_nodes[index].child2;

				m_nodes[index].aabb.merge(m_nodes[child1].aabb, m_nodes[child2].aabb);
				m_nodes[index].height = 1 + max(m_nodes[child1].height, m_nodes[child2].height);

				index = m_nodes[index].parent;
			}
		}
		else
		{
			m_root = sibling;
			m_nodes[sibling].parent = null_node;
			FreeNode(parent);
		}
	}

	int DynamicTree::Balance(int iA)
	{
		cassert(iA != null_node);

		TreeNode* A = m_nodes + iA;
		if (A->IsLeaf() || A->height < 2)
		{
			return iA;
		}

		int iB = A->child1;
		int iC = A->child2;
		cassert(0 <= iB && iB < m_nodeCapacity);
		cassert(0 <= iC && iC < m_nodeCapacity);

		TreeNode* B = m_nodes + iB;
		TreeNode* C = m_nodes + iC;

		int balance = C->height - B->height;

		if (balance > 1)
		{
			int iF = C->child1;
			int iG = C->child2;
			TreeNode* F = m_nodes + iF;
			TreeNode* G = m_nodes + iG;
			cassert(0 <= iF && iF < m_nodeCapacity);
			cassert(0 <= iG && iG < m_nodeCapacity);

			// Swap A and C
			C->child1 = iA;
			C->parent = A->parent;
			A->parent = iC;

			// A's old parent should point to C
			if (C->parent != null_node)
			{
				if (m_nodes[C->parent].child1 == iA)
				{
					m_nodes[C->parent].child1 = iC;
				}
				else
				{
					cassert(m_nodes[C->parent].child2 == iA);
					m_nodes[C->parent].child2 = iC;
				}
			}
			else
			{
				m_root = iC;
			}

			// Rotate
			if (F->height > G->height)
			{
				C->child2 = iF;
				A->child2 = iG;
				G->parent = iA;
				A->aabb.merge(B->aabb, G->aabb);
				C->aabb.merge(A->aabb, F->aabb);

				A->height = 1 + max(B->height, G->height);
				C->height = 1 + max(A->height, F->height);
			}
			else
			{
				C->child2 = iG;
				A->child2 = iF;
				F->parent = iA;
				A->aabb.merge(B->aabb, F->aabb);
				C->aabb.merge(A->aabb, G->aabb);

				A->height = 1 + max(B->height, F->height);
				C->height = 1 + max(A->height, G->height);
			}

			return iC;
		}
		
		if (balance < -1)
		{
			int iD = B->child1;
			int iE = B->child2;
			TreeNode* D = m_nodes + iD;
			TreeNode* E = m_nodes + iE;
			cassert(0 <= iD && iD < m_nodeCapacity);
			cassert(0 <= iE && iE < m_nodeCapacity);

			// Swap A and B
			B->child1 = iA;
			B->parent = A->parent;
			A->parent = iB;

			// A's old parent should point to B
			if (B->parent != null_node)
			{
				if (m_nodes[B->parent].child1 == iA)
				{
					m_nodes[B->parent].child1 = iB;
				}
				else
				{
					cassert(m_nodes[B->parent].child2 == iA);
					m_nodes[B->parent].child2 = iB;
				}
			}
			else
			{
				m_root = iB;
			}

			// Rotate
			if (D->height > E->height)
			{
				B->child2 = iD;
				A->child1 = iE;
				E->parent = iA;
				A->aabb.merge(C->aabb, E->aabb);
				B->aabb.merge(A->aabb, D->aabb);

				A->height = 1 + max(C->height, E->height);
				B->height = 1 + max(A->height, D->height);
			}
			else
			{
				B->child2 = iE;
				A->child1 = iD;
				D->parent = iA;
				A->aabb.merge(C->aabb, D->aabb);
				B->aabb.merge(A->aabb, E->aabb);

				A->height = 1 + max(C->height, D->height);
				B->height = 1 + max(A->height, E->height);
			}

			return iB;
		}

		return iA;
	}

	int DynamicTree::GetHeight() const
	{
		if (m_root == null_node)
			return 0;
		
		return m_nodes[m_root].height;
	}

	float DynamicTree::GetAreaRatio() const
	{
		if (m_root == null_node)
		{
			return 0.0f;
		}

		const TreeNode* root = m_nodes + m_root;
		float rootArea = root->aabb.perimeter();

		float totalArea = 0.0f;
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			const TreeNode* node = m_nodes + i;
			if (node->height < 0)
			{
				// Free node in pool
				continue;
			}

			totalArea += node->aabb.perimeter();
		}

		return totalArea / rootArea;
	}

	// Compute the height of a sub-tree.
	int DynamicTree::ComputeHeight(int nodeId) const
	{
		cassert(0 <= nodeId && nodeId < m_nodeCapacity);
		TreeNode* node = m_nodes + nodeId;

		if (node->IsLeaf())
		{
			return 0;
		}

		int height1 = ComputeHeight(node->child1);
		int height2 = ComputeHeight(node->child2);
		return 1 + max(height1, height2);
	}

	int DynamicTree::ComputeHeight() const
	{
		int height = ComputeHeight(m_root);
		return height;
	}
	
	int DynamicTree::GetMaxBalance() const
	{
		int maxBalance = 0;
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			const TreeNode* node = m_nodes + i;
			if (node->height <= 1)
			{
				continue;
			}

			cassert(node->IsLeaf() == false);

			int child1 = node->child1;
			int child2 = node->child2;
			int balance = fabs(m_nodes[child2].height - m_nodes[child1].height);
			maxBalance = max(maxBalance, balance);
		}

		return maxBalance;
	}

	void DynamicTree::ShiftOrigin(const vec2& newOrigin)
	{
		// Build array of leaves. Free the rest.
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			m_nodes[i].aabb.min -= newOrigin;
			m_nodes[i].aabb.max -= newOrigin;
		}
	}
}