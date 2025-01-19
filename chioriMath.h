#pragma once

#include <cmath>
#include <stdexcept>
#include "commons.h"

namespace chiori
{
	#define c_max(a, b) ((a) > (b) ? (a) : (b))
	#define c_min(a, b) ((a) < (b) ? (a) : (b))
	#define c_abs(A) ((A) > 0.0f ? (A) : -(A))
	#define c_clamp(A, B, C) c_min(c_max(A, B), C)

	constexpr float EPSILON = 1e-7f;		// Epislon value for comparing floats
	constexpr float LEPSILON = 1e-6f;		// Low precision epsilon value
	constexpr float HEPSILON = 1e-8f;		// High precision epsilon value
	constexpr float PI = 3.14159265f;		// Pi
	constexpr float DEG2RAD = PI / 180.0f;	// The value to multiply to a degree value to convert it into radians
	constexpr float RAD2DEG = 180.0f / PI;	// The value to multiply to a radian value to convert it into degrees
	inline bool fltsame(float a, float b) { return (std::abs(a - b) < FLT_EPSILON); }	// float comparison, returns true if a and b are equal within epsilon

	struct cRot
	{
		float s, c; // sin and cos
		constexpr cRot() : s{ 0.0f }, c{ 1.0f } {}
		constexpr cRot(float inS, float inC) : s{ inS }, c{ inC } {}
		cRot(float inRadians) : s{ sin(inRadians) }, c{ cos(inRadians) } {}
		cRot(const cRot& inRHS) : s{ inRHS.s }, c{ inRHS.c } {}
		cRot& operator=(const cRot& inRHS) {
			if (this == &inRHS) return *this;
			s = inRHS.s;
			c = inRHS.c;
			return *this;
		}
		float angle() const { return atan2(s, c); }	// get the angle in radians
		void set(float inRadians) { s = sin(inRadians); c = cos(inRadians); }	// set the angle in radians)
		// Rot operations
		cRot operator-() const { return cRot{ -s, c }; }	// self-negation
		cRot operator*(const cRot& inRHS) const { return cRot{ s * inRHS.c + c * inRHS.s, c * inRHS.c - s * inRHS.s }; }	// rotation composition (multiply)
		cRot operator/(const cRot& inRHS) const { return *this * -inRHS; }	// rotation composition (divide)
		cRot& normalize() // normalize this rotation in place
		{ 
			float m = sqrtf(s * s + c * c);
			float invm = (m > 0) ? 1.0f / m : 0.0f;
			s *= invm;
			c *= invm;
			return *this;
		}	
		cRot normalized() const // normalized version of this rotation
		{
			float m = sqrtf(s * s + c * c);
			float invm = (m > 0) ? 1.0f / m : 0.0f;
			return cRot{ s * invm, c * invm };
		}

		bool operator==(const cRot& inRHS) const {
			return fltsame(s, inRHS.s) && fltsame(c, inRHS.c);
		}
		bool operator!=(const cRot& inRHS) const { return !(*this == inRHS); }

		static const cRot iden;		// s = 0, c = 1
		static const cRot half;		// s = 1, c = 0
		static const cRot zero;		// s = 0, c = 0
	};

	inline constexpr cRot cRot::iden = cRot{ 0.0f, 1.0f };
	inline constexpr cRot cRot::half = cRot{ 1.0f, 0.0f };
	inline constexpr cRot cRot::zero = cRot{ 0.0f, 0.0f };

	struct vec2
	{
		float x, y;

		// Basic class functions
		constexpr vec2() :x{ 0 }, y{ 0 } {}
		constexpr vec2(float inX, float inY) :x{ inX }, y{ inY } {}
		vec2(const vec2& inRHS) : x{ inRHS.x }, y{ inRHS.y } {}
		vec2& operator=(const vec2& inRHS) {
			if (this == &inRHS) return *this;
			x = inRHS.x;
			y = inRHS.y;
			return *this;
		}
		// Vec2 operations
		vec2 operator+(const vec2& inRHS) const { return vec2(x + inRHS.x, y + inRHS.y); }		// component wise addition
		vec2 operator-(const vec2& inRHS) const { return vec2(x - inRHS.x, y - inRHS.y); }		// component wise subtraction
		vec2 operator-() const { return vec2(-x, -y); }											// self-negation
		vec2 operator*(float inScalar) const { return vec2(x * inScalar, y * inScalar); }		// component wise scalar multiplication
		friend vec2 operator*(float inScalar, const vec2& inVec) { return inVec * inScalar; }	// component wise scalar multiplication
		float operator*(const vec2& inRHS) const { return dot(inRHS); }							// dot product
		float dot(const vec2& inRHS) const { return x * inRHS.x + y * inRHS.y; }				// dot product
		float cross(const vec2& inRHS) const { return (x * inRHS.y) - (y * inRHS.x); }			// 2D cross product (returns a scalar)
		vec2 scross(float inScalar) const { return { inScalar * y, -inScalar * x }; }			// 2D scalar cross product (returns a vector)
		vec2 cmult(const vec2& inRHS) const { return {x * inRHS.x, y * inRHS.y}; }				// component wise multiplication with another vector (x*x, y*y)
		vec2 operator/(float inScalar) const { return vec2(x / inScalar, y / inScalar); }	
		vec2 cdiv(const vec2& inRHS) const { return { x / inRHS.x, y / inRHS.y }; }	// component wise scalar division
		friend vec2 operator/(float inScalar, const vec2& inVec) { return inVec / inScalar; }	// component wise scalar division
		float sqrMagnitude() const { return x * x + y * y; }									// square magnitude (self dot product)
		float magnitude() const { return sqrtf(sqrMagnitude()); }							// magnitude (square root of square magnitude, length of the vector)
		vec2 normalized() const { float m = magnitude();  return (m > 0) ? (*this / m) : vec2(); }	// normalized version of this vector
		vec2& normalize() {																		// normalize this vector in place
			float m = magnitude();
			if (m > 0) {
				x /= m;
				y /= m;
			}
			return *this;
		}
		vec2& operator+=(const vec2& inRHS) {			// component wise addition on self
			x += inRHS.x;
			y += inRHS.y;
			return *this;
		}
		vec2& operator-=(const vec2& inRHS) {			// component wise subtraction on self
			x -= inRHS.x;
			y -= inRHS.y;
			return *this;
		}	
		vec2& operator*=(float inScalar) {				// component wise multiplication on self
			x *= inScalar;
			y *= inScalar;
			return *this;
		}
		vec2& operator/=(float inScalar) {				// component wise division on self
			x /= inScalar;
			y /= inScalar;
			return *this;
		}
		// Vec2 index access
		float& operator[](int index) {
			if (index == 0) return x;
			else if (index == 1) return y;
			throw std::out_of_range("[vec2] Index of out range!");
		}
		// Vec2 const index access
		const float& operator[](int index) const {
			if (index == 0) return x;
			else if (index == 1) return y;
			throw std::out_of_range("[vec2] Index of out range!");
		}

		// Vec2 comparisons
		bool operator==(const vec2& inRHS) const {
			return fltsame(x, inRHS.x) && fltsame(y, inRHS.y);
		}
		bool operator!=(const vec2& inRHS) const { return !(*this == inRHS); }
		// Vec2 unique operations
		vec2 rotated(cRot inRot) const			// rotated version of this vector
		{
			return { x * inRot.c - y * inRot.s,
					 x * inRot.s + y * inRot.c };
		}
		vec2& rotate(cRot inRot)				// rotate this vector in place
		{
			float nx = x * inRot.c - y * inRot.s;
			float ny = x * inRot.s + y * inRot.c;
			x = nx; y = ny;
			return *this;
		}
		vec2 tripleCrossProduct(const vec2& inRHS)	// triple cross product (returns a vector)
		{
			//using triple product expansion
			float adotc = sqrMagnitude();
			float bdotc = inRHS.dot(*this);
			return (inRHS * adotc - *this * bdotc);
		}
		// Vec2 debugging
		friend std::ostream& operator<<(std::ostream& inOS, const vec2& inVec) {
			inOS << "vec2(" << inVec.x << ", " << inVec.y << ")";
			return inOS;
		}
		// Static Vec2 properties
		static const vec2 zero;		// x = 0, y = 0
		static const vec2 one;		// x = 1, y = 1
		static const vec2 left;		// x = -1, y = 0
		static const vec2 right;	// x = 1, y = 0
		static const vec2 up;		// x = 0, y = 1
		static const vec2 down;		// x = 0, y = -1

		static vec2 vmin(const vec2& a, const vec2& b)	// component wise minimum of two vectors 
		{
			return { c_min(a.x, b.x), c_min(a.y,b.y) };
		}
		static vec2 vmax(const vec2& a, const vec2& b)	// component wise maximum of two vectors 
		{
			return { c_max(a.x, b.x), c_max(a.y,b.y) };
		}
	};

	inline constexpr vec2 vec2::zero = vec2{ 0.0f, 0.0f };
	inline constexpr vec2 vec2::one = vec2{ 1.0f, 1.0f };
	inline constexpr vec2 vec2::down = vec2{ 0.0f, -1.0f };
	inline constexpr vec2 vec2::left = vec2{ -1.0f, 0.0f };
	inline constexpr vec2 vec2::right = vec2{ 1.0f, 0.0f };
	inline constexpr vec2 vec2::up = vec2{ 0.0f, 1.0f };

	static inline float cross(const vec2& inLHS, const vec2& inRHS)
	{
		return (inLHS.x * inRHS.y) - (inLHS.y * inRHS.x);
	}
	static inline vec2 cross(float inScalar, const vec2& inVec)
	{
		return { -inScalar * inVec.y, inScalar * inVec.x };
	}
	static inline vec2 cross(const vec2& inVec, float inScalar)
	{
		return { inScalar * inVec.y, -inScalar * inVec.x };
	}
	static inline float dot(const vec2& inLHS, const vec2& inRHS)
	{
		return inLHS.dot(inRHS);
	}
	static inline float distance(const vec2& inLHS, const vec2& inRHS)
	{
		vec2 v = inLHS - inRHS;
		return v.magnitude();
	}

	// vector linear interpolation
	static inline vec2 vlerp(vec2 a, vec2 b, float t)
	{
		return { a.x + t * (b.x - a.x), a.y + t * (b.y - a.y) };
	}
	
	struct cTransform 
	{
		cTransform() {}

		cTransform(const vec2& pos, const cRot& rot) : p(pos), q(rot) {}
		//cTransform(const vec2& pos, float rad) : p(pos), q(rad) {}

		void SetIdentity()
		{
			p = vec2::zero;
			q = cRot::iden;
			scale = vec2::one;
		}

		// Set this based on the position and angle (radians).
		void Set(const vec2& position, float angle)
		{
			p = position;
			q.set(angle);
		}

		bool operator==(const cTransform& inRHS) const {
			return p == inRHS.p && scale == inRHS.scale && q == inRHS.q;
		}
		bool operator!=(const cTransform& inRHS) const {
			return !(*this == inRHS);
		}
		
		vec2 p{ vec2::zero };
		cRot q{ cRot::iden };
		vec2 scale{ vec2::one };
	};

	// Transform a vector (e.g local to world)
	inline vec2 cTransformVec(const cTransform& xf, const vec2& v)
	{
		float x = (xf.q.c * v.x - xf.q.s * v.y) * xf.scale.x + xf.p.x;
		float y = (xf.q.s * v.x + xf.q.c * v.y) * xf.scale.y + xf.p.y;
		return { x, y };
	}
	// Inverse transform a vector (e.g world to local)
	inline vec2 cInvTransformVec(const cTransform& xf, const vec2& v)
	{
		float vx = v.x - xf.p.x;
		float vy = v.y - xf.p.y;
		float px = xf.q.c * vx + xf.q.s * vy;
	    float py = -xf.q.s * vx + xf.q.c * vy;
		return { px / xf.scale.x, py / xf.scale.y };
	}
	// Multiply two transforms
	inline cTransform cMulTransforms(const cTransform& A, const cTransform& B)
	{
		cTransform C;
		C.q = A.q * B.q;
		C.p = A.p + B.p.rotated(A.q);
	}
	// Inverse multiply two transforms
	inline cTransform cInvMulTransforms(const cTransform& A, const cTransform& B)
	{
		cTransform C;
		C.q = A.q / B.q;
		C.p = A.p - B.p.rotated(-A.q);
	}
}