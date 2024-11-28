#pragma once
#include "aabbtree.h"

namespace chiori
{
	static constexpr int null_proxy = -1;
	using BroadphaseCallback = std::function<void(void*, void*)>;
	
	struct cPair
	{
		int a;
		int b;
	};
	
	// This will return all the overlapping pairs for the current frame
	// All previous pairs will be overwritten, as such, it is the client's
	// task to track and handle pairs as they require
	class Broadphase
	{
		Broadphase();
		~Broadphase();

		int CreateProxy(const AABB& inAABB, void* inUserData);
		
		void DestroyProxy(int proxyID);

		void MoveProxy(int proxyID, const AABB& inAABB, const vec2& inDisplacement);
		
		void TouchProxy(int proxyID);
		
		const AABB& GetFattenedAABB(int proxyID) const;
		
		void* GetUserData(int proxyID) const;

		const DynamicTree& GetTree() const;

		unsigned GetProxyCount() const;

		void UpdatePairs(BroadphaseCallback callback);

		void Query(const AABB& inAABB, QueryCallback callback);

		void ShiftOrigin(const vec2& inNewOrigin);
		
	private:
		friend class DynamicTree;

		void BufferMove(int proxyID);
		void UnBufferMove(int proxyID);

		bool QueryCallback(int proxyID);

		DynamicTree m_tree;

		unsigned m_proxyCount;

		int* m_moveBuffer;
		int m_moveCapacity;
		int m_moveCount;

		cPair* m_pairBuffer;
		int m_pairCapacity;
		int m_pairCount;

		int m_queryProxyId;
	};

	inline bool cPairLessThan(const cPair& pair1, const cPair& pair2)
	{
		if (pair1.a < pair2.b)
		{
			return true;
		}
		if (pair1.a == pair2.b)
		{
			return pair1.b < pair2.b;
		}
		return false;
	}

	inline void* Broadphase::GetUserData(int proxyId) const
	{
		return m_tree.GetUserData(proxyId);
	}

	inline const AABB& Broadphase::GetFattenedAABB(int proxyId) const
	{
		return m_tree.GetFattenedAABB(proxyId);
	}

	inline unsigned Broadphase::GetProxyCount() const
	{
		return m_proxyCount;
	}

	inline void Broadphase::Query(const AABB& inAABB, chiori::QueryCallback callback)
	{
		m_tree.Query(inAABB, callback);
	}

	inline void Broadphase::ShiftOrigin(const vec2& newOrigin)
	{
		m_tree.ShiftOrigin(newOrigin);
	}
	
	inline const DynamicTree& Broadphase::GetTree() const
	{
		return m_tree;
	}
}