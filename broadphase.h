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
	class cBroadphase
	{
	public:
		cBroadphase();
		~cBroadphase();
		
		int CreateProxy(const cAABB& inAABB, void* inUserData);
		
		void DestroyProxy(int proxyID);

		void MoveProxy(int proxyID, const cAABB& inAABB, const cVec2& inDisplacement);
		
		void TouchProxy(int proxyID);
		
		const cAABB& GetFattenedAABB(int proxyID) const;
		
		void* GetUserData(int proxyID) const;

		const cDynamicTree& GetTree() const;

		unsigned GetProxyCount() const;

		void UpdatePairs(BroadphaseCallback callback);

		void Query(const cAABB& inAABB, QueryCallback callback);

		void ShiftOrigin(const cVec2& inNewOrigin);
		
	private:
		friend class cDynamicTree;

		void BufferMove(int proxyID);
		void UnBufferMove(int proxyID);

		bool QueryCallback(int proxyID);

		cDynamicTree m_tree;

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
		if (pair1.a < pair2.a)
		{
			return true;
		}
		if (pair1.a == pair2.a)
		{
			return pair1.b < pair2.b;
		}
		return false;
	}

	inline void* cBroadphase::GetUserData(int proxyId) const
	{
		return m_tree.GetUserData(proxyId);
	}

	inline const cAABB& cBroadphase::GetFattenedAABB(int proxyId) const
	{
		return m_tree.GetFattenedAABB(proxyId);
	}

	inline unsigned cBroadphase::GetProxyCount() const
	{
		return m_proxyCount;
	}

	inline void cBroadphase::Query(const cAABB& inAABB, chiori::QueryCallback callback)
	{
		m_tree.Query(inAABB, callback);
	}

	inline void cBroadphase::ShiftOrigin(const cVec2& newOrigin)
	{
		m_tree.ShiftOrigin(newOrigin);
	}
	
	inline const cDynamicTree& cBroadphase::GetTree() const
	{
		return m_tree;
	}
	
}