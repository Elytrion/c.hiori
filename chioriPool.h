#pragma once

#include "chioriAllocator.h"
#include <utility> 
#include <unordered_set>

namespace chiori
{
	#define LOOKUP_KEY(x, y) x < y ? (uint64_t)x << 32 | (uint64_t)y : (uint64_t)y << 32 | (uint64_t)x
	inline const int INIT_POOL_SIZE = 16;
	
	/// <summary>
	/// > This is REQUIRED to be an accessible member of the object in the pool <
	/// Acts as a metadata header for quick access to free spots in the pool
	/// while also indexing all the items in the pool for O(1) access by the user
	/// </summary>
	struct cObjHeader
	{
		unsigned index;	// Identifies the position of the object in the pool.
		unsigned next;	// Used for maintaining the free list. When the object is allocated, next points to its own index. When free, next points to the next free slot. 
	};
	
	/// TODO: Make the pool "stable", that is, pointers will never change, by allocating new blocks of memory in new locations and linking the blocks through meta data
	/// This means users can keep pointers to item in the pool and be sure they will never be invalidated by the pool resizing!

	/// <summary>
	/// A memory pool that holds objects of type T. The pool is a contiguous block of memory that is allocated in chunks as needed.
	/// Items are indexed and kept in contiguous memory blocks to ensure cache coherency and fast access times.
	/// </summary>
	/// <typeparam name="T"> The type of object the pool will hold </typeparam>
	//template <typename T>
	//class cPool
	//{
	//public:
	//	static constexpr unsigned invalid_index = static_cast<unsigned>(-1);
	//private:
	//	char* pool;				// Pointer to the raw memory block that holds all objects and metadata.
	//	size_t p_count;			// Number of active (allocated) objects.
	//	size_t p_capacity;		// Total number of objects the pool can hold before needing to grow.
	//	unsigned freeList;		// Index of the first free slot in the pool. Acts as the head of the free linked list.
	//	size_t p_objectSize;	// Size of each object including the cObjHeader metadata
	//	cAllocator* allocator;	// Custom memory allocator used to allocate and deallocate the memory block.

	//	/// <summary>
	//	/// Grows the pool by the specified capacity. Used internally by the pool to double the size to ensure dynamic growth.
	//	/// </summary>
	//	/// <param name="newCapacity"> - The size in number of T objects to increase the pool by </param>
	//	void GrowPool(size_t newCapacity)
	//	{
	//		if (newCapacity <= p_capacity) return;

	//		// Allocate a larger memory block
	//		char* newPool = static_cast<char*>(allocator->allocate(newCapacity * p_objectSize));

	//		// Copy existing pool to the new memory block
	//		if (pool) {
	//			memcpy(newPool, pool, p_capacity * p_objectSize);
	//			allocator->deallocate(pool, p_capacity * p_objectSize);
	//		}

	//		// Initialize new slots in the free list
	//		for (size_t i = p_capacity; i < newCapacity - 1; ++i) {
	//			cObjHeader* obj = reinterpret_cast<cObjHeader*>(newPool + i * p_objectSize);
	//			obj->index = static_cast<unsigned>(i);
	//			obj->next = static_cast<unsigned>(i + 1);
	//		}
	//		cObjHeader* lastObj = reinterpret_cast<cObjHeader*>(newPool + (newCapacity - 1) * p_objectSize);
	//		lastObj->index = static_cast<unsigned>(newCapacity - 1);
	//		lastObj->next = invalid_index; // End of the free list

	//		// Update the free list
	//		if (freeList == invalid_index) {
	//			freeList = static_cast<unsigned>(p_capacity);
	//		}
	//		else {
	//			cObjHeader* oldLast = reinterpret_cast<cObjHeader*>(newPool + (p_capacity - 1) * p_objectSize);
	//			oldLast->next = static_cast<unsigned>(p_capacity);
	//		}

	//		pool = newPool;
	//		p_capacity = newCapacity;
	//	}
	//	
	//	/// <summary>
	//	/// Retrieves an object from the pool based on its index.
	//	/// </summary>
	//	/// <param name="index"> - The index of the object in the pool </param>
	//	/// <returns> The pointer to the object in the pool </returns>
	//	T* GetFromIndex(unsigned index)
	//	{
	//		if (index >= p_capacity) {
	//			throw std::out_of_range("Index out of range");
	//		}
	//		cObjHeader* header = reinterpret_cast<cObjHeader*>(pool + index * p_objectSize);
	//		if (header->next != index) {
	//			throw std::runtime_error("Accessing a freed object");
	//		}
	//		return reinterpret_cast<T*>(header + 1); // Return the object data
	//	}
	//	
	//public:
	//	/// <summary>
	//	/// Initializes the pool
	//	/// </summary>
	//	/// <param name="objectSize"> - sizeof(T) </param>
	//	/// <param name="capacity"> - Initial capacity of the pool </param>
	//	/// <param name="alloc"> - Allocator to use </param>
	//	cPool(cAllocator* alloc, size_t objectSize = sizeof(T), size_t capacity = INIT_POOL_SIZE) :
	//		pool {nullptr}, p_count {0}, p_capacity {0}, freeList(static_cast<unsigned>(-1)),
	//		p_objectSize(objectSize + sizeof(cObjHeader)), allocator(alloc)
	//	{
	//		GrowPool(capacity);
	//	}
	//	
	//	/// <summary>
	//	/// Clears the entire pool at once and frees all the memory
	//	/// </summary>
	//	~cPool()
	//	{
	//		if (pool) {
	//			allocator->deallocate(pool, p_capacity * p_objectSize);
	//		}
	//	}
	//	
	//	/// <summary>
	//	/// Allocates an object from the pool. Will construct the object in-place using placement new.
	//	/// If the free list has no next available slot, it will alloc and grow the pool as needed.
	//	/// </summary>
	//	/// <returns> The pointer to the object in the pool </returns>
	//	T* Alloc()
	//	{
	//		if (freeList == invalid_index) {
	//			GrowPool(p_capacity * 2); // Double the capacity if no free slots
	//		}
	//		cObjHeader* header = reinterpret_cast<cObjHeader*>(pool + freeList * p_objectSize);
	//		freeList = header->next; // Move the free list head
	//		header->next = header->index; // Mark as allocated
	//		++p_count;
	//		return new (header + 1) T(); // Return the object data
	//	}
	//	
	//	/// <summary>
	//	/// Frees an object from the pool.
	//	/// Due to pointer instability, it is recommended to get the pointer via index first, and not store the pointer directly.
	//	/// </summary>
	//	/// <param name="obj"> - The pointer to the object in the pool </param>
	//	void Free(T* obj)
	//	{
	//		if (!obj) return;
	//		cObjHeader* header = reinterpret_cast<cObjHeader*>(reinterpret_cast<char*>(obj) - sizeof(cObjHeader));
	//		cassert(header->index < p_capacity);
	//		cassert(header->next == header->index);
	//		obj->~T();
	//		header->next = freeList; // Add the object back to the free list
	//		freeList = header->index;
	//		--p_count;
	//	}
	//	
	//	/// <summary>
	//	/// Gets the index of a specified object pointer
	//	/// </summary>
	//	/// <param name="obj"> - The pointer to the object in the pool</param>
	//	/// <returns> The index of the object. It is recommended to store this over the object pointer </returns>
	//	unsigned getIndex(T* obj)
	//	{
	//		if (!obj) return -1;
	//		cObjHeader* header = reinterpret_cast<cObjHeader*>(reinterpret_cast<char*>(obj) - sizeof(cObjHeader));
	//		cassert(header->index < p_capacity);
	//		cassert(header->next == header->index);
	//		return header->index;
	//	}
	//	
	//	/// <summary>
	//	/// Gets the number of active (allocated) objects.
	//	/// </summary>
	//	size_t size() const { return p_count; }
	//	
	//	/// <summary>
	//	/// Gets the total number of objects the pool can hold before needing to grow.
	//	/// </summary>
	//	size_t capacity() const { return p_capacity; }
	//	
	//	/// <summary>
	//	/// Provides array-like access to objects in the pool by their index.
	//	/// </summary>
	//	/// <param name="index"> - the index of the object </param>
	//	/// <returns> A pointer to the object in the pool </returns>
	//	T* operator[](int index) { return GetFromIndex(index); }
	//	
	//	/// <summary>
	//	/// Provides const array-like access to objects in the pool by their index.
	//	/// </summary>
	//	/// <param name="index"> - the index of the object </param>
	//	/// <returns> A pointer to the const object in the pool </returns>
	//	const T* operator[](int index) const { return GetFromIndex(index); }
	//	
	//	/// <summary>
	//	/// Gets the actual memory of the pool. It is recommended not to manipulate this due to the addition of metadata.
	//	/// </summary>
	//	char*& data() { return pool; }

	//	/// <summary>
	//	/// Determines if a given index is allocated and thus valid to access
	//	/// This can be used if dynamically freeing and accessing items in the pool
	//	/// </summary>
	//	/// <param name="index"> - the index to check</param>
	//	/// <returns> if the index is allocated and valid </returns>
	//	bool isValid(int index)
	//	{
	//		if (index >= p_capacity || index < 0) {
	//			throw std::out_of_range("Index out of range");
	//		}
	//		cObjHeader* header = reinterpret_cast<cObjHeader*>(pool + index * p_objectSize);
	//		return header->next == header->index;
	//	}
	//};


	template<typename T>
	class cPool
	{
	public:
		static constexpr unsigned invalid_index = static_cast<unsigned>(-1);
	private:
		T* pool;       // Pointer to the array of T
		size_t       p_count;    // Number of active (allocated) objects
		size_t       p_capacity; // Total number of objects the pool can hold
		unsigned     freeList;   // Index of the first free slot
		cAllocator* allocator;	 // Custom memory allocator used to allocate and deallocate the memory block

		void GrowPool(size_t newCapacity)
		{
			if (newCapacity <= p_capacity) return;

			// Allocate a new, larger array of T
			T* newPool = static_cast<T*>(allocator->allocate(newCapacity * sizeof(T)));

			// Copy memory from the old array into the new one 
			// (WARNING: for trivial types only; safe for POD or trivial. 
			//           If T is not trivial, consider manually re-constructing/moving.)
			if (pool)
			{
				std::memcpy(newPool, pool, p_capacity * sizeof(T));
				allocator->deallocate(pool, p_capacity * sizeof(T));
			}

			// Initialize the free slots in [p_capacity .. newCapacity-1]
			for (size_t i = p_capacity; i < newCapacity - 1; ++i)
			{
				newPool[i].header.index = static_cast<unsigned>(i);
				newPool[i].header.next = static_cast<unsigned>(i + 1);
			}
			// The very last newly added object
			newPool[newCapacity - 1].header.index = static_cast<unsigned>(newCapacity - 1);
			newPool[newCapacity - 1].header.next = invalid_index; // end of free list

			// If freeList was invalid before, this means we had no free slots
			// so the new free list starts at p_capacity
			if (freeList == invalid_index)
			{
				freeList = static_cast<unsigned>(p_capacity);
			}
			else
			{
				// Otherwise, link the old chain’s last free object to the new start
				// NOTE: Because we used to store free slots in the old pool,
				//       we assume the last free element in the old pool was at `p_capacity - 1`.
				newPool[p_capacity - 1].header.next = static_cast<unsigned>(p_capacity);
			}

			pool = newPool;
			p_capacity = newCapacity;
		}

		T* GetFromIndex(unsigned index) const
		{
			if (index >= p_capacity)
			{
				throw std::out_of_range("Index out of range");
			}
			T* obj = &pool[index];
			// If next != index, that means this slot is not allocated
			if (obj->header.next != obj->header.index)
			{
				throw std::runtime_error("Accessing a freed object");
			}
			return obj;
		}

	public:
		cPool(cAllocator* alloc, size_t capacity = INIT_POOL_SIZE) :
			pool{ nullptr }, p_count{ 0 }, p_capacity{ 0 }, freeList{ invalid_index }, allocator{ alloc }
		{
			GrowPool(capacity);
		}

		~cPool()
		{
			if (pool)
			{
				allocator->deallocate(pool, p_capacity * sizeof(T));
			}
		}

		T* Alloc()
		{
			// If no free slots, grow
			if (freeList == invalid_index)
			{
				// For simplicity, we just double
				GrowPool(p_capacity * 2);
			}

			// Pop the head of the free list
			T* obj = &pool[freeList];
			freeList = obj->header.next;

			// Placement-new to call T constructor:
			// This will re-construct the entire T including the header,
			// so we’ll forcibly overwrite the index/next immediately after.
			new (obj) T();

			// Ensure the header is in "allocated" state
			unsigned idx = static_cast<unsigned>(obj - pool);
			obj->header.index = idx;
			obj->header.next = idx;  // next == index means "allocated"

			++p_count;
			return obj;
		}

		unsigned getIndex(T* obj) const
		{
			if (!obj) return invalid_index;
			cassert(obj->header.index < p_capacity);
			cassert(obj->header.next == obj->header.index); // must be allocated
			return obj->header.index;
		}

		void Free(T* obj)
		{
			if (!obj) return;

			// Ensure the object is allocated
			cassert(obj->header.index < p_capacity);
			cassert(obj->header.next == obj->header.index);

			// Call the destructor
			obj->~T();

			// Push the object back to the free list
			obj->header.next = freeList;
			freeList = obj->header.index;

			--p_count;
		}

		void Clear()
		{
			// Call the destructor on all allocated objects
			for (size_t i = 0; i < p_capacity; ++i)
			{
				T* obj = &pool[i];
				if (obj->header.next == obj->header.index)
				{
					obj->~T();
				}
			}

			// Reset the free list
			freeList = 0;
			for (size_t i = 0; i < p_capacity - 1; ++i)
			{
				pool[i].header.next = static_cast<unsigned>(i + 1);
			}
			pool[p_capacity - 1].header.next = invalid_index;

			p_count = 0;
		}

		size_t size() const { return p_count; }
		size_t capacity() const { return p_capacity; }
		T* operator[](int index)
		{
			if (index < 0 || static_cast<size_t>(index) >= p_capacity)
			{
				throw std::out_of_range("Index out of range");
			}
			return GetFromIndex(static_cast<unsigned>(index));
		}
		const T* operator[](int index) const
		{
			if (index < 0 || static_cast<size_t>(index) >= p_capacity)
			{
				throw std::out_of_range("Index out of range");
			}
			return GetFromIndex(static_cast<unsigned>(index));
		}

		bool isValid(int index) const
		{
			if (index < 0 || static_cast<size_t>(index) >= p_capacity)
				return false;
			const T* obj = &pool[index];
			return (obj->header.next == obj->header.index);
		}
	};

	// Fast lookup table for pairs of ints, used for pair lookups
	// Just a quick wrapper around unordered_set for the purposes of the project
	class cFLUTable
	{
	private:
		std::unordered_set<uint64_t> data;
	public:
		bool insert(int a, int b) { return insertKey(LOOKUP_KEY(a, b)); }
		bool insertKey(uint64_t key) { return data.emplace(key).second; }
		bool contains(int a, int b) { return containsKey(LOOKUP_KEY(a, b)); }
		bool containsKey(uint64_t key) { return data.find(key) != data.end(); }
		bool erase(int a, int b) { return eraseKey(LOOKUP_KEY(a, b)); }
		bool eraseKey(uint64_t key) { return data.erase(key) > 0; }
		void clear() { data.clear(); }
		size_t size() const { return data.size(); }
		bool empty() const { return data.empty(); }
	};
}