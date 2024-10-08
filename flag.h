#pragma once
#include "pch.h"

namespace chiori
{
	template <typename T>
	class BitFlags
	{
	static_assert(std::is_integral<T>::value, "Only integral types can be used for BitFlags");
	private:
		T _flags;
	public:
		
		BitFlags() : _flags{0} {};
		BitFlags(T inFlags) : _flags{ inFlags } {};

		void set(T inMask) { _flags |= inMask; }
		void clear(T inMask) { _flags &= ~inMask; }
		void reset() { _flags = 0; }
		void toggle(T inMask) { _flags ^= inMask; }
		bool isSet(T inMask) { return (_flags & inMask) == inMask; }
		T get() { return _flags; }
	};

	class Flag_4 : public BitFlags<uint8_t>
	{
	public:
		void set(uint8_t inFlags) { BitFlags::set(inFlags & 0x0F); }
		void clear(uint8_t inFlags) { BitFlags::clear(inFlags & 0x0F); }
		void toggle(uint8_t inFlags) { BitFlags::toggle(inFlags & 0x0F); }
		bool isSet(uint8_t inFlags) { return BitFlags::isSet(inFlags & 0x0F); }
		uint8_t get() { return BitFlags::get() & 0x0F; }
	};
	using Flag_8 = BitFlags<uint8_t>;
	using Flag_16 = BitFlags<uint16_t>;
}