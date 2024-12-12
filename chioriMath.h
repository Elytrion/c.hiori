#pragma once

#include <cmath>
#include <stdexcept>

namespace chiori
{
	constexpr float EPSILON = 1e-6f;		// Epislon value for comparing floats
	constexpr float LEPSILON = 1e-4f;		// Low precision epsilon value
	constexpr float HEPSILON = 1e-8f;		// High precision epsilon value
	constexpr float PI = 3.14159265f;		// Pi
	constexpr float DEG2RAD = PI / 180.0f;	// The value to multiply to a degree value to convert it into radians
	constexpr float RAD2DEG = 180.0f / PI;	// The value to multiply to a radian value to convert it into degrees
	inline bool fltcmp(float a, float b) { return (std::abs(a - b) < FLT_EPSILON); }	// float comparison
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
		vec2 operator/(float inScalar) const { return vec2(x / inScalar, y / inScalar); }		// component wise scalar division
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
			return fltcmp(x, inRHS.x) && fltcmp(y, inRHS.y);
		}
		bool operator!=(const vec2& inRHS) const { return !(*this == inRHS); }
		// Vec2 unique operations
		vec2 rotated(float inRadians) const			// rotated version of this vector
		{
			float rad = inRadians;
			float cosTheta = cos(rad);
			float sinTheta = sin(rad);
			return vec2(x * cosTheta - y * sinTheta, x * sinTheta + y * cosTheta);
		}
		vec2& rotate(float inRadians)				// rotate this vector in place
		{
			float rad = inRadians;
			float cosTheta = cos(rad);
			float sinTheta = sin(rad);
			float nx = x * cosTheta - y * sinTheta;
			float ny = x * sinTheta + y * cosTheta;
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
			return { min(a.x, b.x), min(a.y,b.y) };
		}

		static vec2 vmax(const vec2& a, const vec2& b)	// component wise maximum of two vectors 
		{
			return { max(a.x, b.x), max(a.y,b.y) };
		}
	};

	inline constexpr vec2 vec2::zero = vec2{ 0.0f, 0.0f };
	inline constexpr vec2 vec2::one = vec2{ 1.0f, 1.0f };
	inline constexpr vec2 vec2::down = vec2{ 0.0f, -1.0f };
	inline constexpr vec2 vec2::left = vec2{ -1.0f, 0.0f };
	inline constexpr vec2 vec2::right = vec2{ 1.0f, 0.0f };
	inline constexpr vec2 vec2::up = vec2{ 0.0f, 1.0f };

	
	struct cTransform 
	{
		cTransform() {}

		cTransform(const vec2& position, const float& rotation) : pos(position), rot(rotation) {}

		void SetIdentity()
		{
			pos = vec2::zero;
			rot = 0.0f;
			scale = vec2::zero;
		}

		// Set this based on the position and angle (radians).
		void Set(const vec2& position, float angle)
		{
			pos = position;
			rot = angle;
		}

		bool operator==(const cTransform& inRHS) const {
			return pos == inRHS.pos && scale == inRHS.scale && rot == inRHS.rot;
		}
		bool operator!=(const cTransform& inRHS) const {
			return !(*this == inRHS);
		}
		
		vec2 pos{ vec2::zero };
		vec2 scale{ vec2::one };
		float rot{ 0.0f };
	};

	static inline vec2 cTransformVec(const cTransform& xf, const vec2& v)
	{
		vec2 nv = v.rotated(xf.rot);
		nv += xf.pos;
		nv = nv.cmult(xf.scale);
		return nv;
	}


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
}