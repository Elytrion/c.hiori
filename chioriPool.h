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
	
	template <typename T, typename Allocator = std::allocator<char>>
	class cPool
	{
	public:
		static constexpr unsigned invalid_index = static_cast<unsigned>(-1);
	private:
		char* pool;
		size_t p_count;
		size_t p_capacity;
		unsigned freeList;
		size_t objectSize;
		Allocator allocator;

		void GrowPool(size_t newCapacity)
		{
			if (newCapacity <= p_capacity) return;

			// Allocate a larger memory block
			char* newPool = allocator.allocate(newCapacity * objectSize);

			// Copy existing pool to the new memory block
			if (pool) {
				memcpy(newPool, pool, p_capacity * objectSize);
				allocator.deallocate(pool, p_capacity * objectSize);
			}

			// Initialize new slots in the free list
			for (size_t i = p_capacity; i < newCapacity - 1; ++i) {
				cObjHeader* obj = reinterpret_cast<cObjHeader*>(newPool + i * objectSize);
				obj->index = static_cast<unsigned>(i);
				obj->next = static_cast<unsigned>(i + 1);
			}
			cObjHeader* lastObj = reinterpret_cast<cObjHeader*>(newPool + (newCapacity - 1) * objectSize);
			lastObj->index = static_cast<unsigned>(newCapacity - 1);
			lastObj->next = invalid_index; // End of the free list

			// Update the free list
			if (freeList == invalid_index) {
				freeList = static_cast<unsigned>(p_capacity);
			}
			else {
				cObjHeader* oldLast = reinterpret_cast<cObjHeader*>(newPool + (p_capacity - 1) * objectSize);
				oldLast->next = static_cast<unsigned>(p_capacity);
			}

			pool = newPool;
			p_capacity = newCapacity;
		}
		
		T* GetFromIndex(unsigned index)
		{
			if (index >= p_capacity) {
				throw std::out_of_range("Index out of range");
			}
			cObjHeader* header = reinterpret_cast<cObjHeader*>(pool + index * objectSize);
			if (header->next != index) {
				throw std::runtime_error("Accessing a freed object");
			}
			return reinterpret_cast<T*>(header + 1); // Return the object data
		}
		
	public:
		cPool(size_t objectSize = sizeof(T), size_t capacity = 16, const Allocator& alloc = Allocator()) :
			pool {nullptr}, p_count {0}, p_capacity {0}, freeList(static_cast<unsigned>(-1)),
			objectSize(objectSize + sizeof(cObjHeader)), allocator(alloc)
		{
			GrowPool(capacity);
		}
		
		~cPool()
		{
			if (pool) {
				allocator.deallocate(pool, p_capacity * objectSize);
			}
		}

		T* Alloc()
		{
			if (freeList == invalid_index) {
				GrowPool(p_capacity * 2); // Double the capacity if no free slots
			}
			cObjHeader* header = reinterpret_cast<cObjHeader*>(pool + freeList * objectSize);
			freeList = header->next; // Move the free list head
			header->next = header->index; // Mark as allocated
			++p_count;
			return reinterpret_cast<T*>(header + 1); // Return the object data
		}
		
		void Free(T* obj)
		{
			if (!obj) return;
			cObjHeader* header = reinterpret_cast<cObjHeader*>(static_cast<char*>(obj) - sizeof(cObjHeader));
			cassert(header->index < p_capacity);
			cassert(header->next == header->index);
			header->next = freeList; // Add the object back to the free list
			freeList = header->index;
			--p_count;
		}

		size_t size() const { return p_count; }
		
		size_t capacity() const { return p_capacity; }

		T* operator[](int indx) { return GetFromIndex(indx); }

		const T* operator[](int indx) const { return GetFromIndex(indx); }
	};
}