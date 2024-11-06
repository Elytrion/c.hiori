#include "pch.h"
#include "cActor.h"

namespace chiori
{
	void cActor::setFlags(Flag_8 inFlags)
	{
		_flags = inFlags;
	}
	void cActor::setFlags(int inFlags)
	{
		_flags.set(inFlags);
	}
}