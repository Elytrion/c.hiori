#include "pch.h"
#include "broadphase.h"
#include <functional>

namespace chiori
{
	cBroadphase::cBroadphase()
	{
		m_proxyCount = 0;

		m_pairCapacity = 16;
		m_pairCount = 0;
		m_pairBuffer = new cPair[m_pairCapacity];

		m_moveCapacity = 16;
		m_moveCount = 0;
		m_moveBuffer = new int[m_moveCapacity];
	}
	
	cBroadphase::~cBroadphase()
	{
		delete[] m_pairBuffer;
		delete[] m_moveBuffer;
	}

	int cBroadphase::CreateProxy(const AABB& aabb, void* userData)
	{
		int proxyId = m_tree.InsertProxy(aabb, userData);
		++m_proxyCount;
		BufferMove(proxyId);
		return proxyId;
	}

	void cBroadphase::DestroyProxy(int proxyId)
	{
		UnBufferMove(proxyId);
		--m_proxyCount;
		m_tree.DestroyProxy(proxyId);
	}

	void cBroadphase::MoveProxy(int proxyId, const AABB& aabb, const cVec2& displacement)
	{
		bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
		if (buffer)
		{
			BufferMove(proxyId);
		}
	}

	void cBroadphase::TouchProxy(int proxyId)
	{
		BufferMove(proxyId);
	}

	void cBroadphase::BufferMove(int proxyId)
	{
		if (m_moveCount == m_moveCapacity)
		{
			int* oldBuffer = m_moveBuffer;
			m_moveCapacity *= 2;
			m_moveBuffer = new int[m_moveCapacity];
			memcpy(m_moveBuffer, oldBuffer, m_moveCount * sizeof(int));
			delete[] oldBuffer;
		}

		m_moveBuffer[m_moveCount] = proxyId;
		++m_moveCount;
	}

	void cBroadphase::UnBufferMove(int proxyId)
	{
		for (int i = 0; i < m_moveCount; ++i)
		{
			if (m_moveBuffer[i] == proxyId)
				m_moveBuffer[i] = null_proxy;
		}
	}

	bool cBroadphase::QueryCallback(int proxyId)
	{
		// A proxy cannot form a pair with itself.
		if (proxyId == m_queryProxyId)
		{
			return true;
		}

		// Grow the pair buffer as needed.
		if (m_pairCount == m_pairCapacity)
		{
			cPair* oldBuffer = m_pairBuffer;
			m_pairCapacity *= 2;
			m_pairBuffer = new cPair[m_pairCapacity];
			memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(cPair));
			delete[] oldBuffer;
		}

		m_pairBuffer[m_pairCount].a = min(proxyId, m_queryProxyId);
		m_pairBuffer[m_pairCount].b = max(proxyId, m_queryProxyId);
		cPair test1 = m_pairBuffer[0];
		cPair test2 = m_pairBuffer[1];
		++m_pairCount;

		return true;
	}

	void cBroadphase::UpdatePairs(BroadphaseCallback callback)
	{
		// Reset pair buffer
		m_pairCount = 0;

		// Perform tree queries for all moving proxies.
		for (int i = 0; i < m_moveCount; ++i)
		{

			m_queryProxyId = m_moveBuffer[i];
			if (m_queryProxyId == null_proxy)
			{
				continue;
			}

			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			const AABB& fatAABB = m_tree.GetFattenedAABB(m_queryProxyId);

			// Query tree, create pairs and add them pair buffer.
			auto q_callback = [this](unsigned int proxyId) -> bool {
				return this->QueryCallback(proxyId);
				};
			m_tree.Query(fatAABB, q_callback);
		}

		// Reset move buffer
		m_moveCount = 0;

		// Sort the pair buffer to expose duplicates.
		std::sort(m_pairBuffer, m_pairBuffer + m_pairCount, cPairLessThan);

		// Send the pairs back to the client.
		int i = 0;
		while (i < m_pairCount)
		{
			cPair* primaryPair = m_pairBuffer + i;
			void* userDataA = m_tree.GetUserData(primaryPair->a);
			void* userDataB = m_tree.GetUserData(primaryPair->b);

			callback(userDataA, userDataB); // Gives the cilent the new pair
			++i;

			// Skip any duplicate pairs.
			while (i < m_pairCount)
			{
				cPair* pair = m_pairBuffer + i;
				if (pair->a != primaryPair->a || pair->b != primaryPair->b)
				{
					break;
				}
				++i;
			}
		}
	}
}