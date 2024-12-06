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
	public:
		using invalid_index = static_cast<unsigned>(-1);
	private:
		char* pool;
		size_t count;
		size_t capacity;
		unsigned freeList;
		size_t objectSize;
		Allocator alloc;

		void GrowPool(size_t newCapacity)
		{
			if (newCapacity <= capacity) return;

			// Allocate a larger memory block
			char* newPool = alloc.allocate(newCapacity * objectSize);

			// Copy existing pool to the new memory block
			if (pool) {
				memcpy(newPool, pool, capacity * objectSize);
				alloc.deallocate(pool, capacity * objectSize);
			}

			// Initialize new slots in the free list
			for (size_t i = capacity; i < newCapacity - 1; ++i) {
				cObjHeader* obj = reinterpret_cast<cObjHeader*>(newPool + i * objectSize);
				obj->index = static_cast<unsigned>(i);
				obj->next = static_cast<unsigned>(i + 1);
			}
			cObjHeader* lastObj = reinterpret_cast<cObjHeader*>(newPool + (newCapacity - 1) * objectSize);
			lastObj->index = static_cast<unsigned>(newCapacity - 1);
			lastObj->next = invalid_index; // End of the free list

			// Update the free list
			if (freeList == invalid_index) {
				freeList = static_cast<unsigned>(capacity);
			}
			else {
				cObjHeader* oldLast = reinterpret_cast<cObjHeader*>(newPool + (capacity - 1) * objectSize);
				oldLast->next = static_cast<unsigned>(capacity);
			}

			pool = newPool;
			capacity = newCapacity;
		}
		
		void* GetFromIndex(unsigned index)
		{
			if (index >= capacity) {
				throw std::out_of_range("Index out of range");
			}
			cObjHeader* header = reinterpret_cast<cObjHeader*>(pool + index * objectSize);
			if (header->next != index) {
				throw std::runtime_error("Accessing a freed object");
			}
			return reinterpret_cast<void*>(header + 1); // Return the object data
		}
		
	public:
		cPool(size_t objectSize, size_t capacity, const Allocator& alloc = Allocator()) :
			pool {nullptr}, count {0}, capacity {0}, freeList(static_cast<unsigned>(-1)),
			objectSize(objSize + sizeof(cObjHeader)), allocator(alloc)
		{
			GrowPool(capacity);
		}
		
		~cPool()
		{
			if (pool) {
				allocator.deallocate(pool, capacity * objectSize);
			}
		}

		void* Alloc()
		{
			if (freeList == invalid_index) {
				GrowPool(capacity * 2); // Double the capacity if no free slots
			}
			cObjHeader* header = reinterpret_cast<cObjHeader*>(pool + freeList * objectSize);
			freeList = header->next; // Move the free list head
			header->next = header->index; // Mark as allocated
			++count;
			return reinterpret_cast<void*>(header + 1); // Return the object data
		}
		
		void Free(void* obj)
		{
			if (!obj) return;
			cObjHeader* header = reinterpret_cast<cObjHeader*>(static_cast<char*>(obj) - sizeof(cObjHeader));
			cassert(header->index < capacity);
			cassert(header->next == header->index);
			header->next = freeList; // Add the object back to the free list
			freeList = header->index;
			--count;
		}

		size_t size() const { return count; }
		
		size_t capacity() const { return capacity; }

		void* operator[](int indx) { return GetFromIndex(indx); }

		const void* operator[](int indx) const { return GetFromIndex(indx); }
	};
}