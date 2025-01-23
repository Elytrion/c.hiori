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
		
		/// Multiply two rotations: b * a
		/// equivalent to angle addition via trig identity:
		///	sin(b + a) = sin(b) * cos(a) + cos(b) * sin(a)
		///	cos(b + a) = cos(b) * cos(a) - sin(b) * sin(a)
		///	order independent!
		cRot mulRot(const cRot& b) const
		{
			return{ b.s * c + b.c * s,
					b.c * c - b.s * s };
		}
		/// Transpose multiply two rotations: inv(b) * a
		/// equivalent to angle subtraction via trig identity:
		///	sin(a - b) = sin(b) * cos(a) - cos(b) * sin(a)
		///	cos(a - b) = cos(b) * cos(a) + sin(b) * sin(a)
		cRot invMulRot(const cRot& b) const
		{
			return { b.c * s - b.s * c,
					 b.c * c + b.s * s };
		}

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
		cRot& intergrate(float omegah) // integrate this rotation with the given angular velocity (omega) multiplied by time step (h) in place
		{
			s = s + omegah * c;
			c = c - omegah * s;
			this->normalize();
			return *this;
		}
		cRot intergrated(float omegah)
		{
			cRot r = { s + omegah * c, c - omegah * s };
			return r.normalized();
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

	struct cVec2
	{
		float x, y;

		// Basic class functions
		constexpr cVec2() :x{ 0 }, y{ 0 } {}
		constexpr cVec2(float inX, float inY) :x{ inX }, y{ inY } {}
		cVec2(const cVec2& inRHS) : x{ inRHS.x }, y{ inRHS.y } {}
		cVec2& operator=(const cVec2& inRHS) {
			if (this == &inRHS) return *this;
			x = inRHS.x;
			y = inRHS.y;
			return *this;
		}
		// Vec2 operations
		cVec2 operator+(const cVec2& inRHS) const { return cVec2(x + inRHS.x, y + inRHS.y); }		// component wise addition
		cVec2 operator-(const cVec2& inRHS) const { return cVec2(x - inRHS.x, y - inRHS.y); }		// component wise subtraction
		cVec2 operator-() const { return cVec2(-x, -y); }											// self-negation
		cVec2 operator*(float inScalar) const { return cVec2(x * inScalar, y * inScalar); }		// component wise scalar multiplication
		friend cVec2 operator*(float inScalar, const cVec2& inVec) { return inVec * inScalar; }	// component wise scalar multiplication
		float operator*(const cVec2& inRHS) const { return dot(inRHS); }							// dot product
		float dot(const cVec2& inRHS) const { return x * inRHS.x + y * inRHS.y; }				// dot product
		float cross(const cVec2& inRHS) const { return (x * inRHS.y) - (y * inRHS.x); }			// 2D cross product (returns a scalar)
		cVec2 scross(float inScalar) const { return { inScalar * y, -inScalar * x }; }			// 2D scalar cross product (returns a vector)
		cVec2 cmult(const cVec2& inRHS) const { return {x * inRHS.x, y * inRHS.y}; }				// component wise multiplication with another vector (x*x, y*y)
		cVec2 operator/(float inScalar) const { return cVec2(x / inScalar, y / inScalar); }	
		cVec2 cdiv(const cVec2& inRHS) const { return { x / inRHS.x, y / inRHS.y }; }	// component wise scalar division
		friend cVec2 operator/(float inScalar, const cVec2& inVec) { return inVec / inScalar; }	// component wise scalar division
		float sqrMagnitude() const { return x * x + y * y; }									// square magnitude (self dot product)
		float magnitude() const { return sqrtf(sqrMagnitude()); }							// magnitude (square root of square magnitude, length of the vector)
		cVec2 normalized() const { float m = magnitude();  return (m > 0) ? (*this / m) : cVec2(); }	// normalized version of this vector
		cVec2& normalize() {																		// normalize this vector in place
			float m = magnitude();
			if (m > 0) {
				x /= m;
				y /= m;
			}
			return *this;
		}
		cVec2& operator+=(const cVec2& inRHS) {			// component wise addition on self
			x += inRHS.x;
			y += inRHS.y;
			return *this;
		}
		cVec2& operator-=(const cVec2& inRHS) {			// component wise subtraction on self
			x -= inRHS.x;
			y -= inRHS.y;
			return *this;
		}	
		cVec2& operator*=(float inScalar) {				// component wise multiplication on self
			x *= inScalar;
			y *= inScalar;
			return *this;
		}
		cVec2& operator/=(float inScalar) {				// component wise division on self
			x /= inScalar;
			y /= inScalar;
			return *this;
		}
		// Vec2 index access
		float& operator[](int index) {
			if (index == 0) return x;
			else if (index == 1) return y;
			throw std::out_of_range("[cVec2] Index of out range!");
		}
		// Vec2 const index access
		const float& operator[](int index) const {
			if (index == 0) return x;
			else if (index == 1) return y;
			throw std::out_of_range("[cVec2] Index of out range!");
		}

		// Vec2 comparisons
		bool operator==(const cVec2& inRHS) const {
			return fltsame(x, inRHS.x) && fltsame(y, inRHS.y);
		}
		bool operator!=(const cVec2& inRHS) const { return !(*this == inRHS); }
		// Vec2 unique operations
		cVec2 rotated(cRot inRot) const			// rotated version of this vector
		{
			return { x * inRot.c - y * inRot.s,
					 x * inRot.s + y * inRot.c };
		}
		cVec2& rotate(cRot inRot)				// rotate this vector in place
		{
			float nx = x * inRot.c - y * inRot.s;
			float ny = x * inRot.s + y * inRot.c;
			x = nx; y = ny;
			return *this;
		}
		cVec2 tripleCrossProduct(const cVec2& inRHS)	// triple cross product (returns a vector)
		{
			//using triple product expansion
			float adotc = sqrMagnitude();
			float bdotc = inRHS.dot(*this);
			return (inRHS * adotc - *this * bdotc);
		}
		// Vec2 debugging
		friend std::ostream& operator<<(std::ostream& inOS, const cVec2& inVec) {
			inOS << "cVec2(" << inVec.x << ", " << inVec.y << ")";
			return inOS;
		}
		// Static Vec2 properties
		static const cVec2 zero;		// x = 0, y = 0
		static const cVec2 one;		// x = 1, y = 1
		static const cVec2 left;		// x = -1, y = 0
		static const cVec2 right;	// x = 1, y = 0
		static const cVec2 up;		// x = 0, y = 1
		static const cVec2 down;		// x = 0, y = -1

		static cVec2 vmin(const cVec2& a, const cVec2& b)	// component wise minimum of two vectors 
		{
			return { c_min(a.x, b.x), c_min(a.y,b.y) };
		}
		static cVec2 vmax(const cVec2& a, const cVec2& b)	// component wise maximum of two vectors 
		{
			return { c_max(a.x, b.x), c_max(a.y,b.y) };
		}
	};

	inline constexpr cVec2 cVec2::zero = cVec2{ 0.0f, 0.0f };
	inline constexpr cVec2 cVec2::one = cVec2{ 1.0f, 1.0f };
	inline constexpr cVec2 cVec2::down = cVec2{ 0.0f, -1.0f };
	inline constexpr cVec2 cVec2::left = cVec2{ -1.0f, 0.0f };
	inline constexpr cVec2 cVec2::right = cVec2{ 1.0f, 0.0f };
	inline constexpr cVec2 cVec2::up = cVec2{ 0.0f, 1.0f };

	static inline float cross(const cVec2& inLHS, const cVec2& inRHS)
	{
		return (inLHS.x * inRHS.y) - (inLHS.y * inRHS.x);
	}
	static inline cVec2 cross(float inScalar, const cVec2& inVec)
	{
		return { -inScalar * inVec.y, inScalar * inVec.x };
	}
	static inline cVec2 cross(const cVec2& inVec, float inScalar)
	{
		return { inScalar * inVec.y, -inScalar * inVec.x };
	}
	static inline float dot(const cVec2& inLHS, const cVec2& inRHS)
	{
		return inLHS.dot(inRHS);
	}
	static inline float distance(const cVec2& inLHS, const cVec2& inRHS)
	{
		cVec2 v = inLHS - inRHS;
		return v.magnitude();
	}

	// vector linear interpolation
	static inline cVec2 vlerp(cVec2 a, cVec2 b, float t)
	{
		return { a.x + t * (b.x - a.x), a.y + t * (b.y - a.y) };
	}
	
	struct cTransform 
	{
		cTransform() {}

		cTransform(const cVec2& pos, const cRot& rot) : p(pos), q(rot) {}
		//cTransform(const cVec2& pos, float rad) : p(pos), q(rad) {}

		void SetIdentity()
		{
			p = cVec2::zero;
			q = cRot::iden;
			scale = cVec2::one;
		}

		// Set this based on the position and angle (radians).
		void Set(const cVec2& position, float angle)
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
		
		cVec2 p{ cVec2::zero };
		cRot q{ cRot::iden };
		cVec2 scale{ cVec2::one };

	};
	
	// Transform a vector (e.g local to world)
	inline cVec2 cTransformVec(const cTransform& xf, const cVec2& v)
	{
		float x = (xf.q.c * v.x - xf.q.s * v.y) * xf.scale.x + xf.p.x;
		float y = (xf.q.s * v.x + xf.q.c * v.y) * xf.scale.y + xf.p.y;
		return { x, y };
	}
	// Inverse transform a vector (e.g world to local)
	inline cVec2 cInvTransformVec(const cTransform& xf, const cVec2& v)
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
		C.q = B.q.mulRot(A.q);
		C.p = A.p + B.p.rotated(A.q);
		return C;
	}
	// Inverse multiply two transforms
	inline cTransform cInvMulTransforms(const cTransform& A, const cTransform& B)
	{
		cTransform C;
		C.q = B.q.invMulRot(A.q);
		C.p = (B.p - A.p).rotated(-A.q);
		return C;
	}
}