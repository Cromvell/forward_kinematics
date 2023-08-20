#ifndef MATRIX_H
#define MATRIX_H
#include <initializer_list>
#include <iterator>
#include <cstring>
#include <QDebug>
#include <cassert>


template <typename T>
struct Vector4;

template <typename T>
struct Vector3 {
    T x, y, z;
    static const size_t SIZE = 3;

    T operator*(const Vector3 &v) {
        return x * v.x + y * v.y + z * v.z;
    }
    T &operator[](const size_t index) {
        switch (index) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        default: assert(false);
        }
    }

    static Vector3<T> zeros() {
        return Vector3<T>{0.0, 0.0, 0.0};
    }

    static Vector3<T> from_homogenious(Vector4<T> &v) {
        return Vector3<T>{v.x / v.w, v.y / v.w, v.z / v.w};
    }
};

template <typename T>
struct Vector4 {
    T x, y, z, w;
    static const size_t SIZE = 4;

    T operator*(const Vector4 &v) {
        return x * v.x + y * v.y + z * v.z + w * v.w;
    }
    T &operator[](const size_t index) {
        switch (index) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        case 3: return w;
        default: assert(false);
        }
    }

    static Vector4<T> zeros() {
        return Vector4<T>{0.0, 0.0, 0.0, 0.0};
    }

    static Vector4<T> zeros_homogenious() {
        return Vector4<T>{0.0, 0.0, 0.0, 1.0};
    }

    static Vector4<T> to_homogenious(Vector3<T> &v) {
        return Vector4<T>{v.x, v.y, v.z, 1.0};
    }
};

template<class T>
struct Matrix4 final {
    T _data[4][4];
    static const size_t ROWS = 4;
    static const size_t COLS = 4;
    static constexpr size_t TOTAL_ELEMENT_COUNT = ROWS * COLS;

    Matrix4() { this->set_to_zero(); }
    Matrix4(std::initializer_list<T> l) {
        assert(l.size() == TOTAL_ELEMENT_COUNT);
        this->set_to_zero();
        for (auto it = l.begin(); it != l.end(); ++it) {
            size_t linear_index = std::distance(l.begin(), it);
            size_t i = linear_index / COLS;
            size_t j = linear_index % COLS;
            _data[i][j] = *it;
        }
    }
    Matrix4(std::initializer_list<std::initializer_list<T>> l) {
        assert(l.size() == ROWS);
        assert((*l.begin()).size() == COLS);
        this->set_to_zero();
        for (auto it1 = l.begin(); it1 != l.end(); ++it1) {
            size_t i = std::distance( l.begin(), it1);
            auto &inner_l = *it1;
            for (auto it2 = inner_l.begin(); it2 != inner_l.end(); ++it2) {
                size_t j = std::distance(inner_l.begin(), it2);
                _data[i][j] = *it2;
            }
        }
    }
    Matrix4(const Matrix4 &oth) {
        std::memcpy(_data, oth._data, TOTAL_ELEMENT_COUNT * sizeof(T));
    }
    Matrix4 &operator=(const Matrix4 &oth) {
        if (this == &oth)  return *this;

        std::memcpy(_data, oth._data, TOTAL_ELEMENT_COUNT * sizeof(T));
        return *this;
    }

    T &operator[](const size_t i) {
        return _data[i / COLS][i % COLS];
    }

    Vector4<T> operator*(const Vector4<T> &v) {
        Vector4<T> result = Vector4<T>::zeros();
        Vector4<T> v_o{v.x, v.y, v.z, v.w};

        for (size_t i = 0; i < v.SIZE; i++) {
            Vector4<T> row{_data[i][0], _data[i][1], _data[i][2], _data[i][3]};
            result[i] = row * v_o;
        }
        return result;
    }

    void set_to_zero() {
        std::memset(_data, 0, TOTAL_ELEMENT_COUNT * sizeof(T));
    }

    static Matrix4<T> eye() {
        Matrix4<T> m{{1.0, 0.0, 0.0, 0.0},
                     {0.0, 1.0, 0.0, 0.0},
                     {0.0, 0.0, 1.0, 0.0},
                     {0.0, 0.0, 0.0, 1.0}};
        return m;
    }
};

typedef Vector3<float> Vector3f;
typedef Vector4<float> Vector4f;
typedef Matrix4<float> Matrix4f;

template <typename T>
Matrix4<T> matmul(Matrix4<T> &m1, Matrix4<T> &m2) {
    Matrix4<T> result;
    for (size_t i = 0; i < m1.ROWS; i++) {
        for (size_t j = 0; j < m1.COLS; j++) {
            T acc = m1[i * m1.ROWS] * m2[j];
            for (size_t k = 1; k < m1.COLS; k++) {
                acc += m1[i * m1.ROWS + k] * m2[k * m2.ROWS + j];
            }
            result[i * m1.COLS + j] = acc;
        }
    }
    return result;
}


template <class t>
QDebug operator<<(QDebug d, Matrix4<t>& m) {
    d << "[" << m._data[0][0] << " " << m._data[0][1] << " " << m._data[0][2] << " " << m._data[0][3] << "]\n"
      << "[" << m._data[1][0] << " " << m._data[1][1] << " " << m._data[1][2] << " " << m._data[1][3] << "]\n"
      << "[" << m._data[2][0] << " " << m._data[2][1] << " " << m._data[2][2] << " " << m._data[2][3] << "]\n"
      << "[" << m._data[3][0] << " " << m._data[3][1] << " " << m._data[3][2] << " " << m._data[3][3] << "]\n";
    return d;
}

template <class t>
std::ostream &operator<<(std::ostream &s, Matrix4<t>& m) {
    s << "[" << m._data[0][0] << " " << m._data[0][1] << " " << m._data[0][2] << " " << m._data[0][3] << "]\n"
      << "[" << m._data[1][0] << " " << m._data[1][1] << " " << m._data[1][2] << " " << m._data[1][3] << "]\n"
      << "[" << m._data[2][0] << " " << m._data[2][1] << " " << m._data[2][2] << " " << m._data[2][3] << "]\n"
      << "[" << m._data[3][0] << " " << m._data[3][1] << " " << m._data[3][2] << " " << m._data[3][3] << "]\n";
    return s;
}

template <class t>
QDebug operator<<(QDebug d, Vector4<t>& v) {
    d << "[" << v.x << " " << v.y << " " << v.z << " " << v.w <<  "]\n";
    return d;
}

template <class t>
QDebug operator<<(QDebug d, Vector3<t>& v) {
    d << "[" << v.x << " " << v.y << " " << v.z <<  "]\n";
    return d;
}

#endif // MATRIX_H
