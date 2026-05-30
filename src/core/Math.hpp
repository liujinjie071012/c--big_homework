#pragma once

#include <cmath>

// 2D 向量模板类
template <typename T>
class Vec2 {
public:
    T x, y;

    // 构造函数
    Vec2() : x(0), y(0) {}
    Vec2(T x, T y) : x(x), y(y) {}

    // 基础算术运算符重载
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(T s) const { return Vec2(x * s, y * s); }
    Vec2 operator/(T s) const { 
        if (s == 0) return Vec2(0, 0);
        return Vec2(x / s, y / s); 
    }
    Vec2 operator-() const { return Vec2(-x, -y); }

    // 复合赋值运算符重载
    Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
    Vec2& operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }
    Vec2& operator*=(T s) { x *= s; y *= s; return *this; }
    Vec2& operator/=(T s) { 
        if (s != 0) { x /= s; y /= s; } 
        return *this; 
    }

    // 比较运算符
    bool operator==(const Vec2& v) const { return x == v.x && y == v.y; }
    bool operator!=(const Vec2& v) const { return !(*this == v); }

    // 点乘运算
    T dot(const Vec2& v) const { return x * v.x + y * v.y; }
    
    // 叉乘运算：2D 向量叉乘返回标量 (大小代表平行四边形面积，正负代表方向)
    T cross(const Vec2& v) const { return x * v.y - y * v.x; }

    // 叉乘运算扩展：向量与标量叉乘得到垂直向量
    Vec2 cross(T s) const { return Vec2(s * y, -s * x); }

    // 长度与归一化
    T lengthSquared() const { return x * x + y * y; }
    T length() const { return std::sqrt(x * x + y * y); }
    
    Vec2 normalized() const {
        T len = length();
        if (len > 0) {
            return Vec2(x / len, y / len);
        }
        return Vec2(0, 0);
    }

    void normalize() {
        T len = length();
        if (len > 0) {
            x /= len;
            y /= len;
        }
    }

    // 旋转向量 (弧度)
    Vec2 rotated(T angle) const {
        T c = std::cos(angle);
        T s = std::sin(angle);
        return Vec2(x * c - y * s, x * s + y * c);
    }

    // 计算到另一个向量的距离
    T distance(const Vec2& v) const {
        return (*this - v).length();
    }
    T distanceSquared(const Vec2& v) const {
        return (*this - v).lengthSquared();
    }
};

// 标量乘向量的全局运算符重载
template <typename T>
inline Vec2<T> operator*(T s, const Vec2<T>& v) {
    return Vec2<T>(v.x * s, v.y * s);
}

// 叉乘的便捷全局函数
template <typename T>
inline T dot(const Vec2<T>& a, const Vec2<T>& b) {
    return a.dot(b);
}

template <typename T>
inline T cross(const Vec2<T>& a, const Vec2<T>& b) {
    return a.cross(b);
}

template <typename T>
inline Vec2<T> cross(const Vec2<T>& v, T s) {
    return v.cross(s);
}

template <typename T>
inline Vec2<T> cross(T s, const Vec2<T>& v) {
    return Vec2<T>(-s * v.y, s * v.x);
}


// 2D 矩阵模板类
template <typename T>
class Mat2 {
public:
    // [ m00  m01 ]
    // [ m10  m11 ]
    T m00, m01;
    T m10, m11;

    // 默认构造单位矩阵
    Mat2() : m00(1), m01(0), m10(0), m11(1) {}
    
    // 参数构造
    Mat2(T m00, T m01, T m10, T m11) : m00(m00), m01(m01), m10(m10), m11(m11) {}

    // 根据旋转弧度构造旋转矩阵
    Mat2(T radians) {
        T c = std::cos(radians);
        T s = std::sin(radians);
        m00 = c; m01 = -s;
        m10 = s; m11 = c;
    }

    // 矩阵乘向量 (进行坐标旋转变换)
    Vec2<T> operator*(const Vec2<T>& v) const {
        return Vec2<T>(m00 * v.x + m01 * v.y, m10 * v.x + m11 * v.y);
    }

    // 矩阵乘矩阵
    Mat2 operator*(const Mat2& o) const {
        return Mat2(
            m00 * o.m00 + m01 * o.m10, m00 * o.m01 + m01 * o.m11,
            m10 * o.m00 + m11 * o.m10, m10 * o.m01 + m11 * o.m11
        );
    }

    // 转置矩阵
    Mat2 transpose() const {
        return Mat2(m00, m10, m01, m11);
    }

    // 逆矩阵
    Mat2 inverse() const {
        T det = m00 * m11 - m01 * m10;
        if (det == 0) {
            return Mat2(); // 如果行列式为0，返回单位阵
        }
        T invDet = 1 / det;
        return Mat2(
            m11 * invDet, -m01 * invDet,
            -m10 * invDet, m00 * invDet
        );
    }

    // 设置旋转角度
    void set(T radians) {
        T c = std::cos(radians);
        T s = std::sin(radians);
        m00 = c; m01 = -s;
        m10 = s; m11 = c;
    }
};

using Vec2f = Vec2<float>;
using Mat2f = Mat2<float>;
