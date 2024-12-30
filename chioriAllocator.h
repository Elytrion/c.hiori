#pragma once
#include <memory>
#include <stdexcept>

namespace chiori
{
	// interface
	struct cAllocator
	{
		virtual ~cAllocator() = default;
		virtual void* allocate(size_t inSize) = 0;
		virtual void deallocate(void* inPtr, size_t inSize) = 0;
	};

	template <typename Allocator>
	class cAllocatorWrapper : public cAllocator
	{
	private:
		Allocator m_allocator;
	public:
		explicit cAllocatorWrapper(Allocator alloc = Allocator{}) : m_allocator(std::move(alloc)) {}

		void* allocate(size_t inSize) override
		{
			return m_allocator.allocate(inSize);
		}

		void deallocate(void* inPtr, size_t inSize) override
		{
			m_allocator.deallocate(inPtr, inSize);
		}
	};

	struct cDefaultAllocator // any custom allocator has to implement the allocate and deallocate functions
	{
		void* allocate(size_t inSize)
		{
			return ::operator new(inSize);
		}

		void deallocate(void* inPtr, size_t inSize)
		{
			::operator delete(inPtr);
		}
	};
}