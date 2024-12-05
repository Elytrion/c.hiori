#pragma once

#include <cmath>
#include <stdexcept>

namespace chiori
{
	constexpr float EPSILON = 1e-5f;		// Epislon value for comparing floats
	constexpr float LEPSILON = 1e-3f;		// Low precision epsilon value
	constexpr float HEPSILON = 1e-7f;		// High precision epsilon value
	constexpr float PI = 3.14159265f;		// Pi
	constexpr float DEG2RAD = PI / 180.0f;	// The value to multiply to a degree value to convert it into radians
	constexpr float RAD2DEG = 180.0f / PI;	// The value to multiply to a radian value to convert it into degrees
	
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
		vec2 operator+(const vec2& inRHS) const { return vec2(x + inRHS.x, y + inRHS.y); }
		vec2 operator-(const vec2& inRHS) const { return vec2(x - inRHS.x, y - inRHS.y); }
		vec2 operator-() const { return vec2(-x, -y); }
		vec2 operator*(float inScalar) const { return vec2(x * inScalar, y * inScalar); }
		friend vec2 operator*(float inScalar, const vec2& inVec) { return inVec * inScalar; }
		float operator*(const vec2& inRHS) const { return dot(inRHS); }
		float dot(const vec2& inRHS) const { return x * inRHS.x + y * inRHS.y; }
		float cross(const vec2& inRHS) const { return (x * inRHS.y) - (y * inRHS.x); }
		vec2 scross(float inScalar) const { return { inScalar * y, -inScalar * x }; }
		vec2 operator/(float inScalar) const { return vec2(x / inScalar, y / inScalar); }
		friend vec2 operator/(float inScalar, const vec2& inVec) { return inVec / inScalar; }
		float sqrMagnitude() const { return x * x + y * y; }
		float magnitude() const { return std::sqrt(sqrMagnitude()); }
		vec2 normalized() const { float m = magnitude();  return (m > 0) ? (*this / m) : vec2(); }
		vec2& normalize() {
			float m = magnitude();
			if (m > 0) {
				x /= m;
				y /= m;
			}
			return *this;
		}
		vec2& operator+=(const vec2& inRHS) {
			x += inRHS.x;
			y += inRHS.y;
			return *this;
		}
		vec2& operator-=(const vec2& inRHS) {
			x -= inRHS.x;
			y -= inRHS.y;
			return *this;
		}
		vec2& operator*=(float inScalar) {
			x *= inScalar;
			y *= inScalar;
			return *this;
		}
		vec2& operator/=(float inScalar) {
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
		const float& operator[](int index) const {
			if (index == 0) return x;
			else if (index == 1) return y;
			throw std::out_of_range("[vec2] Index of out range!");
		}

		// Vec2 comparisons
		bool operator==(const vec2& inRHS) const {
			bool xsame = (std::abs(x - inRHS.x) < EPSILON);
			bool ysame = (std::abs(y - inRHS.y) < EPSILON);
			return xsame && ysame;
		}
		bool operator!=(const vec2& inRHS) const { return !(*this == inRHS); }
		// Vec2 unique operations
		vec2 rotated(float inDegrees) const
		{
			float rad = inDegrees * DEG2RAD;
			float cosTheta = cos(rad);
			float sinTheta = sin(rad);
			return vec2(x * cosTheta - y * sinTheta, x * sinTheta + y * cosTheta);
		}
		vec2& rotate(float inDegrees)
		{
			float rad = inDegrees * DEG2RAD;
			float cosTheta = cos(rad);
			float sinTheta = sin(rad);
			float nx = x * cosTheta - y * sinTheta;
			float ny = x * sinTheta + y * cosTheta;
			x = nx; y = ny;
			return *this;
		}
		vec2 tripleCrossProduct(const vec2& inRHS)
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
		static const vec2 zero;
		static const vec2 one;
		static const vec2 left;
		static const vec2 right;
		static const vec2 up;
		static const vec2 down;

		static vec2 vmin(const vec2& a, const vec2& b)
		{
			return { min(a.x, b.x), min(a.y,b.y) };
		}

		static vec2 vmax(const vec2& a, const vec2& b)
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
	
	struct cRot
	{
		cRot() {}
		// all angles in radians
		explicit cRot(float angle)
		{
			s = sinf(angle);
			c = cosf(angle);
		}

		void Set(float angle)
		{
			s = sinf(angle);
			c = cosf(angle);
		}

		void SetIdentity()
		{
			s = 0.0f;
			c = 1.0f;
		}

		// Get the angle in radians
		float GetAngle() const { return std::atan2(s, c); }
		
		float s, c; //sin, cos
	};
	
	struct cTransform 
	{
		cTransform() {}

		cTransform(const vec2& position, const cRot& rotation) : p(position), r(rotation) {}

		void SetIdentity()
		{
			p = vec2::zero;
			r.SetIdentity();
		}

		// Set this based on the position and angle (radians).
		void Set(const vec2& position, float angle)
		{
			p = position;
			r.Set(angle);
		}
		
		vec2 p;
		cRot r;
	};
}