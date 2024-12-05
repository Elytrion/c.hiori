#pragma once

#include <memory>
#include <stdexcept>
#include <utility> 

namespace chiori
{
	struct cObjHeader
	{
		unsigned index;
		unsigned next;
	};
	
	template <typename Allocator = std::allocator<char>>
	class cPool
	{
	private:
		char* pool;
		size_t count;
		size_t capacity;
		unsigned freeList;
		size_t objectSize;
		Allocator alloc;

		void GrowPool();
		void* GetFromIndex(unsigned index);
		
	public:
		cPool(size_t objectSize, size_t capacity);
		~cPool();

		void* Alloc();
		void Free(void* obj);

		size_t size() const;
		size_t capacity() const;
	};
}