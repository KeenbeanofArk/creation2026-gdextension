#pragma once
#include <cmath>
#include <cstring>

// ============================================================
// Vec3
// ============================================================
struct Vec3 {
	float x = 0.0f, y = 0.0f, z = 0.0f;

	Vec3() = default;
	Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

	Vec3 operator+(Vec3 b) const { return { x + b.x, y + b.y, z + b.z }; }
	Vec3 operator-(Vec3 b) const { return { x - b.x, y - b.y, z - b.z }; }
	Vec3 operator*(float s) const { return { x * s, y * s, z * s }; }
	Vec3 operator/(float s) const {
		float inv = 1.0f / s;
		return { x * inv, y * inv, z * inv };
	}
	Vec3 operator-() const { return { -x, -y, -z }; }
	Vec3 &operator+=(Vec3 b) {
		x += b.x;
		y += b.y;
		z += b.z;
		return *this;
	}
	Vec3 &operator-=(Vec3 b) {
		x -= b.x;
		y -= b.y;
		z -= b.z;
		return *this;
	}
	Vec3 &operator*=(float s) {
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}
	bool operator==(Vec3 b) const { return x == b.x && y == b.y && z == b.z; }

	static float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	static Vec3 cross(Vec3 a, Vec3 b) {
		return { a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x };
	}

	float length_sq() const { return x * x + y * y + z * z; }
	float length() const { return std::sqrt(length_sq()); }

	Vec3 normalized() const {
		float len = length();
		if (len < 1e-8f)
			return { 0.0f, 0.0f, 0.0f };
		return *this * (1.0f / len);
	}

	bool is_near_zero() const { return length_sq() < 1e-10f; }
	bool has_nan() const { return std::isnan(x) || std::isnan(y) || std::isnan(z); }

	static Vec3 zero() { return { 0.0f, 0.0f, 0.0f }; }
	static Vec3 up() { return { 0.0f, 1.0f, 0.0f }; }
	static Vec3 forward() { return { 0.0f, 0.0f, -1.0f }; }
	static Vec3 right() { return { 1.0f, 0.0f, 0.0f }; }
};

inline Vec3 operator*(float s, Vec3 v) { return v * s; }

// ============================================================
// Quat
// ============================================================
struct Quat {
	float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

	Quat() = default;
	Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

	static Quat identity() { return { 0.0f, 0.0f, 0.0f, 1.0f }; }

	static Quat from_axis_angle(Vec3 axis, float angle_rad) {
		Vec3 a = axis.normalized();
		float s = std::sin(angle_rad * 0.5f);
		float c = std::cos(angle_rad * 0.5f);
		return { a.x * s, a.y * s, a.z * s, c };
	}

	// Hamilton product: this * b
	Quat operator*(Quat b) const {
		return {
			w * b.x + x * b.w + y * b.z - z * b.y,
			w * b.y - x * b.z + y * b.w + z * b.x,
			w * b.z + x * b.y - y * b.x + z * b.w,
			w * b.w - x * b.x - y * b.y - z * b.z
		};
	}

	// Rotate a vector by this quaternion (assumes unit quaternion)
	Vec3 rotate(Vec3 v) const {
		// q * (0,v) * q^-1  optimised form
		Vec3 u{ x, y, z };
		float s = w;
		return u * (2.0f * Vec3::dot(u, v)) + v * (s * s - Vec3::dot(u, u)) + Vec3::cross(u, v) * (2.0f * s);
	}

	Quat conjugate() const { return { -x, -y, -z, w }; }

	float dot(Quat b) const { return x * b.x + y * b.y + z * b.z + w * b.w; }

	float length_sq() const { return x * x + y * y + z * z + w * w; }
	float length() const { return std::sqrt(length_sq()); }

	Quat normalized() const {
		float len = length();
		if (len < 1e-8f)
			return identity();
		float inv = 1.0f / len;
		return { x * inv, y * inv, z * inv, w * inv };
	}

	// Integrate angular velocity over dt
	static Quat integrate(Quat q, Vec3 ang_vel, float dt) {
		// q' = q + 0.5 * dt * w_quat * q
		Quat omega{ ang_vel.x * 0.5f * dt,
			ang_vel.y * 0.5f * dt,
			ang_vel.z * 0.5f * dt,
			0.0f };
		Quat delta = omega * q;
		Quat result{ q.x + delta.x, q.y + delta.y, q.z + delta.z, q.w + delta.w };
		return result.normalized();
	}

	static Quat slerp(Quat a, Quat b, float t) {
		float d = a.dot(b);
		// Ensure shortest path
		if (d < 0.0f) {
			b = { -b.x, -b.y, -b.z, -b.w };
			d = -d;
		}
		if (d > 0.9995f) {
			// Linear interpolation fallback
			Quat r{ a.x + t * (b.x - a.x),
				a.y + t * (b.y - a.y),
				a.z + t * (b.z - a.z),
				a.w + t * (b.w - a.w) };
			return r.normalized();
		}
		float theta0 = std::acos(d);
		float theta = theta0 * t;
		float s0 = std::cos(theta) - d * std::sin(theta) / std::sin(theta0);
		float s1 = std::sin(theta) / std::sin(theta0);
		return { s0 * a.x + s1 * b.x,
			s0 * a.y + s1 * b.y,
			s0 * a.z + s1 * b.z,
			s0 * a.w + s1 * b.w };
	}

	bool has_nan() const { return std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w); }

	// Convert to Euler angles (YXZ order, radians)
	Vec3 to_euler_yxz() const {
		Quat n = normalized();
		float sinr_cosp = 2.0f * (n.w * n.x + n.y * n.z);
		float cosr_cosp = 1.0f - 2.0f * (n.x * n.x + n.y * n.y);
		float pitch = std::atan2(sinr_cosp, cosr_cosp);

		float sinp = 2.0f * (n.w * n.y - n.z * n.x);
		float yaw;
		if (std::abs(sinp) >= 1.0f)
			yaw = std::copysign(3.14159265f * 0.5f, sinp);
		else
			yaw = std::asin(sinp);

		float siny_cosp = 2.0f * (n.w * n.z + n.x * n.y);
		float cosy_cosp = 1.0f - 2.0f * (n.y * n.y + n.z * n.z);
		float roll = std::atan2(siny_cosp, cosy_cosp);

		return { pitch, yaw, roll };
	}
};

// ============================================================
// Mat3   (row-major: m[row*3 + col])
// ============================================================
struct Mat3 {
	float m[9] = {};

	Mat3() { std::memset(m, 0, sizeof(m)); }

	float &at(int r, int c) { return m[r * 3 + c]; }
	float at(int r, int c) const { return m[r * 3 + c]; }

	static Mat3 identity() {
		Mat3 M;
		M.m[0] = M.m[4] = M.m[8] = 1.0f;
		return M;
	}

	static Mat3 zero() { return Mat3{}; }

	static Mat3 from_quat(Quat q) {
		Quat n = q.normalized();
		float xx = n.x * n.x, yy = n.y * n.y, zz = n.z * n.z;
		float xy = n.x * n.y, xz = n.x * n.z, yz = n.y * n.z;
		float wx = n.w * n.x, wy = n.w * n.y, wz = n.w * n.z;
		Mat3 R;
		R.m[0] = 1.0f - 2.0f * (yy + zz);
		R.m[1] = 2.0f * (xy - wz);
		R.m[2] = 2.0f * (xz + wy);
		R.m[3] = 2.0f * (xy + wz);
		R.m[4] = 1.0f - 2.0f * (xx + zz);
		R.m[5] = 2.0f * (yz - wx);
		R.m[6] = 2.0f * (xz - wy);
		R.m[7] = 2.0f * (yz + wx);
		R.m[8] = 1.0f - 2.0f * (xx + yy);
		return R;
	}

	static Mat3 outer_product(Vec3 a, Vec3 b) {
		Mat3 M;
		M.m[0] = a.x * b.x;
		M.m[1] = a.x * b.y;
		M.m[2] = a.x * b.z;
		M.m[3] = a.y * b.x;
		M.m[4] = a.y * b.y;
		M.m[5] = a.y * b.z;
		M.m[6] = a.z * b.x;
		M.m[7] = a.z * b.y;
		M.m[8] = a.z * b.z;
		return M;
	}

	Mat3 operator+(Mat3 b) const {
		Mat3 R;
		for (int i = 0; i < 9; ++i)
			R.m[i] = m[i] + b.m[i];
		return R;
	}
	Mat3 operator-(Mat3 b) const {
		Mat3 R;
		for (int i = 0; i < 9; ++i)
			R.m[i] = m[i] - b.m[i];
		return R;
	}
	Mat3 operator*(float s) const {
		Mat3 R;
		for (int i = 0; i < 9; ++i)
			R.m[i] = m[i] * s;
		return R;
	}

	Mat3 operator*(Mat3 b) const {
		Mat3 R;
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				R.at(r, c) = at(r, 0) * b.at(0, c) + at(r, 1) * b.at(1, c) + at(r, 2) * b.at(2, c);
		return R;
	}

	Vec3 operator*(Vec3 v) const {
		return {
			m[0] * v.x + m[1] * v.y + m[2] * v.z,
			m[3] * v.x + m[4] * v.y + m[5] * v.z,
			m[6] * v.x + m[7] * v.y + m[8] * v.z
		};
	}

	Mat3 transpose() const {
		Mat3 R;
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				R.at(r, c) = at(c, r);
		return R;
	}

	// Cramer's rule inverse (valid for non-singular 3x3)
	Mat3 inverse() const {
		float c00 = m[4] * m[8] - m[5] * m[7];
		float c01 = m[5] * m[6] - m[3] * m[8];
		float c02 = m[3] * m[7] - m[4] * m[6];
		float det = m[0] * c00 + m[1] * c01 + m[2] * c02;
		if (std::abs(det) < 1e-10f)
			return zero();
		float inv = 1.0f / det;
		Mat3 R;
		R.m[0] = c00 * inv;
		R.m[1] = (m[2] * m[7] - m[1] * m[8]) * inv;
		R.m[2] = (m[1] * m[5] - m[2] * m[4]) * inv;
		R.m[3] = c01 * inv;
		R.m[4] = (m[0] * m[8] - m[2] * m[6]) * inv;
		R.m[5] = (m[2] * m[3] - m[0] * m[5]) * inv;
		R.m[6] = c02 * inv;
		R.m[7] = (m[1] * m[6] - m[0] * m[7]) * inv;
		R.m[8] = (m[0] * m[4] - m[1] * m[3]) * inv;
		return R;
	}

	// Rotate inertia tensor from body to world: R * I * R^T
	Mat3 rotated_by(Quat q) const {
		Mat3 R = Mat3::from_quat(q);
		return R * (*this) * R.transpose();
	}

	// --- Inertia tensors (diagonal, body-space, about centre of mass) ---
	static Mat3 inertia_sphere(float mass, float radius) {
		float v = 0.4f * mass * radius * radius;
		Mat3 I;
		I.m[0] = I.m[4] = I.m[8] = v;
		return I;
	}

	static Mat3 inertia_box(float mass, float hx, float hy, float hz) {
		// half-extents
		Mat3 I;
		I.m[0] = (1.0f / 3.0f) * mass * (hy * hy + hz * hz);
		I.m[4] = (1.0f / 3.0f) * mass * (hx * hx + hz * hz);
		I.m[8] = (1.0f / 3.0f) * mass * (hx * hx + hy * hy);
		return I;
	}

	static Mat3 inertia_capsule(float mass, float radius, float half_height) {
		// Approximate as cylinder + 2 hemispheres
		float h = half_height * 2.0f;
		float r2 = radius * radius;
		float ixx = mass * (3.0f * r2 + h * h) / 12.0f;
		float iyy = 0.5f * mass * r2;
		Mat3 I;
		I.m[0] = ixx;
		I.m[4] = iyy;
		I.m[8] = ixx;
		return I;
	}
};
