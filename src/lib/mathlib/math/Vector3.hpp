/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Will Perone <will.perone@gmail.com>
 *           Anton Babushkin <anton.babushkin@me.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Vector3.hpp
 *
 * 3D Vector
 */

#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include <math.h>
#include "../CMSIS/Include/arm_math.h"

namespace math
{

template <typename T>
class Vector3 {
public:
	T x, y, z;
	arm_matrix_instance_f32 arm_col;

	/**
	 * trivial ctor
	 */
	Vector3<T>() {
		arm_col = {3, 1, &x};
	}

	/**
	 * setting ctor
	 */
	Vector3<T>(const T x0, const T y0,  const T z0): x(x0), y(y0), z(z0) {
		arm_col = {3, 1, &x};
	}

	/**
	 * setting ctor
	 */
	Vector3<T>(const T data[3]): x(data[0]), y(data[1]), z(data[2]) {
		arm_col = {3, 1, &x};
	}

	/**
	 * setter
	 */
	void set(const T x0, const T y0, const T z0) {
		x = x0;
		y = y0;
		z = z0;
	}

	/**
	 * access to elements by index
	 */
	T operator ()(unsigned int i) {
		return *(&x + i);
	}

	/**
	 * access to elements by index
	 */
	const T operator ()(unsigned int i) const {
		return *(&x + i);
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Vector3<T> &v) {
		return (x == v.x && y == v.y && z == v.z);
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Vector3<T> &v) {
		return (x != v.x || y != v.y || z != v.z);
	}

	/**
	 * set to value
	 */
	const Vector3<T> &operator =(const Vector3<T> &v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}

	/**
	 * negation
	 */
	const Vector3<T> operator -(void) const {
		return Vector3<T>(-x, -y, -z);
	}

	/**
	 * addition
	 */
	const Vector3<T> operator +(const Vector3<T> &v) const {
		return Vector3<T>(x + v.x, y + v.y, z + v.z);
	}

	/**
	 * subtraction
	 */
	const Vector3<T> operator -(const Vector3<T> &v) const {
		return Vector3<T>(x - v.x, y - v.y, z - v.z);
	}

	/**
	 * uniform scaling
	 */
	const Vector3<T> operator *(const T num) const {
		Vector3<T> temp(*this);
		return temp *= num;
	}

	/**
	 * uniform scaling
	 */
	const Vector3<T> operator /(const T num) const {
		Vector3<T> temp(*this);
		return temp /= num;
	}

	/**
	 * addition
	 */
	const Vector3<T> &operator +=(const Vector3<T> &v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	/**
	 * subtraction
	 */
	const Vector3<T> &operator -=(const Vector3<T> &v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector3<T> &operator *=(const T num) {
		x *= num;
		y *= num;
		z *= num;
		return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector3<T> &operator /=(const T num) {
		x /= num;
		y /= num;
		z /= num;
		return *this;
	}

	/**
	 * dot product
	 */
	T operator *(const Vector3<T> &v) const {
		return x * v.x + y * v.y + z * v.z;
	}

	/**
	 * cross product
	 */
	const Vector3<T> operator %(const Vector3<T> &v) const {
		Vector3<T> temp(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
		return temp;
	}

	/**
	 * gets the length of this vector squared
	 */
	float length_squared() const {
		return (*this * *this);
	}

	/**
	 * gets the length of this vector
	 */
	float length() const {
		return (T)sqrt(*this * *this);
	}

	/**
	 * normalizes this vector
	 */
	void normalize() {
		*this /= length();
	}

	/**
	 * returns the normalized version of this vector
	 */
	Vector3<T> normalized() const {
		return *this / length();
	}

	/**
	 * reflects this vector about n
	 */
	void reflect(const Vector3<T> &n)
	{
		Vector3<T> orig(*this);
		project(n);
		*this = *this * 2 - orig;
	}

	/**
	 * projects this vector onto v
	 */
	void project(const Vector3<T> &v) {
		*this = v * (*this * v) / (v * v);
	}

	/**
	 * returns this vector projected onto v
	 */
	Vector3<T> projected(const Vector3<T> &v) {
		return v * (*this * v) / (v * v);
	}

	/**
	 * computes the angle between 2 arbitrary vectors
	 */
	static inline float angle(const Vector3<T> &v1, const Vector3<T> &v2) {
		return acosf((v1 * v2) / (v1.length() * v2.length()));
	}

	/**
	 * computes the angle between 2 arbitrary normalized vectors
	 */
	static inline float angle_normalized(const Vector3<T> &v1, const Vector3<T> &v2) {
		return acosf(v1 * v2);
	}
};

typedef Vector3<float> Vector3f;
}

#endif // VECTOR3_HPP
