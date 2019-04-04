/*
 * Copyright © 2019 Andrea Bontempi All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * - Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * - Neither the name of Andrea Bontempi nor the names of its contributors may be used to
 *   endorse or promote products derived from this software without specific prior
 *   written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef QUATER_H
#define QUATER_H

#include <cmath>
#include <type_traits>
#include <complex>

template<typename T = double>
class Quaternion {
    
private:
    
    T n, ni, nj, nk;
    
public:
    
    using value_type = T; ///< value_type trait for STL compatibility
    
    /**
     * Default constuctor
     */
    Quaternion(const T& n = static_cast<T>(0), const T& ni = static_cast<T>(0), const T& nj = static_cast<T>(0), const T& nk = static_cast<T>(0))
        : n(n), ni(ni), nj(nj), nk(nk) {}

    /**
     * Specialized constructor for std::complex
     */
    Quaternion(const std::complex<T>& complex_a, const std::complex<T>& complex_b = std::complex<T>(static_cast<T>(0), static_cast<T>(0)))
        : n(complex_a.real()), ni(complex_a.imag()), nj(complex_b.real()), nk(complex_b.imag()) {}
    
    /**
     * Copy constructor 
     */
    template<typename U>
    Quaternion(const Quaternion<U>& rhs)
        : n(rhs.a()), ni(rhs.b()), nj(rhs.c()), nk(rhs.d()) {}
    
    /**
     * Default copy assignment operator
     */
    template<typename U>
    Quaternion<T>& operator=(const Quaternion<U>& rhs) {
        this->n = rhs.a();
        this->ni = rhs.b();
        this->nj = rhs.c();
        this->nk = rhs.d();
    }
    
    /**
     * Specialized copy assignment operator for std::complex
     */
    template<typename U>
    Quaternion<T>& operator=(const std::complex<U> rhs) {
        this->n = rhs.real();
        this->ni = rhs.imag();
        this->nj = static_cast<T>(0);
        this->nk = static_cast<T>(0);
    }
    
    T a() const {
        return this->n;
    }
    
    T b() const {
        return this->ni;
    }
    
    T c() const {
        return this->nj;
    }
    
    T d() const {
        return this->nk;
    }
    
    std::complex<T> complex_a() const {
        return {this->n, this->ni};
    }
    
    std::complex<T> complex_b() const {
        return {this->nj, this->nk};
    }
    
    T real() const {
        return this->n;
    }
    
    Quaternion<T> unreal() const {
        return {static_cast<T>(0), this->ni, this->nj, this->nk};
    }
       
};

namespace std {
    
    /**
     * Norm of quaternion
     */
    template<typename T>
    T norm(const Quaternion<T>& quat) {
        return (quat.a() * quat.a()) + (quat.b() * quat.b()) + (quat.c() * quat.c()) + (quat.d() * quat.d());
    }
    
    /**
    * Implementation of abs with float sqrt 
    */
    float abs(const Quaternion<float>& quat) {
        return std::sqrt(std::norm(quat));
    }

    /**
    * Implementation of abs with double sqrt 
    */
    double abs(const Quaternion<double>& quat) {
        return std::sqrt(std::norm(quat));
    }

    /**
    * Implementation of abs with long double sqrt 
    */
    long double abs(const Quaternion<long double>& quat) {
        return std::sqrt(std::norm(quat));
    }
    
    /**
    * The conjugate of quaternion
    */
    template<typename T>
    Quaternion<T> conj(const Quaternion<T>& quat) {
        return {quat.a(), quat.b() * -1, quat.c() * -1, quat.d() * -1};
    }
    
    /**
     * Is not a number?
     */
    template<typename T>
    bool isnan(Quaternion<T> quat) {
        return std::isnan(quat.a()) || std::isnan(quat.b()) || std::isnan(quat.c()) || std::isnan(quat.d());
    }
    
    /**
     * Is infinite?
     */
    template<typename T>
    bool isinf(Quaternion<T> quat) {
        return std::isinf(quat.a()) || std::isinf(quat.b()) || std::isinf(quat.c()) || std::isinf(quat.d());
    }
    
    /**
     * Is finite?
     */
    template<typename T>
    bool isfinite(Quaternion<T> quat) {
        return std::isfinite(quat.a()) && std::isfinite(quat.b()) && std::isfinite(quat.c()) && std::isfinite(quat.d());
    }
    
}

/**
 * Add operator between two quaternios.
 */
template<typename _tA, typename _tB>
auto operator+(const Quaternion<_tA>& lhs, const Quaternion<_tB>& rhs) -> Quaternion<decltype(lhs.a() + rhs.a())> {
    return {lhs.a() + rhs.a(), lhs.b() + rhs.b(), lhs.c() + rhs.c(), lhs.d() + rhs.d()};
}

/**
 * Add operator between quaternion and std::complex.
 */
template<typename _tA, typename _tB>
auto operator+(const Quaternion<_tA>& lhs, const std::complex<_tB>& rhs) -> Quaternion<decltype(lhs.a() + rhs.real())> {
    return {lhs.a() + rhs.real(), lhs.b() + rhs.imag(), lhs.c(), lhs.d()};
}

/**
 * Add operator between std::complex and quaternion.
 */
template<typename _tA, typename _tB>
auto operator+(const std::complex<_tB>& lhs, const Quaternion<_tA>& rhs) -> Quaternion<decltype(lhs.real() + rhs.a())> {
    return operator+(rhs, lhs);
}

/**
 * Add operator between quaternion and scalar.
 */
template<typename _tA, typename _tB>
auto operator+(const Quaternion<_tA>& lhs, const _tB& rhs) -> Quaternion<decltype(lhs.a() + rhs)> {
    return {lhs.a() + rhs, lhs.b(), lhs.c(), lhs.d()};
}

/**
 * Add operator between scalar and quaternion.
 */
template<typename _tA, typename _tB>
auto operator+(const _tA& lhs, const Quaternion<_tB>& rhs) -> Quaternion<decltype(lhs + rhs.a())> {
    return operator+(rhs, lhs);
}

/**
 * Sub operator between two quaternions.
 */
template<typename _tA, typename _tB>
auto operator-(const Quaternion<_tA>& lhs, const Quaternion<_tB>& rhs) -> Quaternion<decltype(lhs.a() - rhs.a())> {
    return {lhs.a() - rhs.a(), lhs.b() - rhs.b(), lhs.c() - rhs.c(), lhs.d() - rhs.d()};
}

/**
 * Sub operator between quaternion and std:complex.
 */
template<typename _tA, typename _tB>
auto operator-(const Quaternion<_tA>& lhs, const std::complex<_tB>& rhs) -> Quaternion<decltype(lhs.a() - rhs.real())> {
    return {lhs.a() - rhs.real(), lhs.b() - rhs.imag(), lhs.c(), lhs.d()};
}

/**
 * Sub operator between std:complex and quaternion.
 */
template<typename _tA, typename _tB>
auto operator-(const std::complex<_tB>& lhs, const Quaternion<_tA>& rhs) -> Quaternion<decltype(lhs.real() - rhs.a())> {
    return {lhs.real() - rhs.a(), lhs.imag() - rhs.b(), -rhs.c(), -rhs.d()};
}

/**
 * Sub operator between quaternion and scalar.
 */
template<typename _tA, typename _tB>
auto operator-(const Quaternion<_tA>& lhs, const _tB& rhs) -> Quaternion<decltype(lhs.a() - rhs)> {
    return {lhs.a() - rhs, lhs.b(), lhs.c(), lhs.d()};
}

/**
 * Sub operator between scalar and quaternion.
 */
template<typename _tA, typename _tB>
auto operator-(const _tA& lhs, const Quaternion<_tB>& rhs) -> Quaternion<decltype(lhs - rhs.a())> {
    return {lhs - rhs.a(), -rhs.b(), -rhs.c(), -rhs.d()};
}

/**
 * Mul operator between two quaternions.
 */
template<typename _tA, typename _tB>
auto operator*(const Quaternion<_tA>& lhs, const Quaternion<_tB>& rhs) -> Quaternion<decltype(lhs.a() * rhs.a())> {
    decltype(lhs.a() * rhs.a()) tn = (lhs.a() * rhs.a()) - (lhs.b() * rhs.b()) - (lhs.c() * rhs.c()) - (lhs.d() * rhs.d());
    decltype(lhs.a() * rhs.a()) tni = (lhs.a() * rhs.b()) + (lhs.b() * rhs.a()) + (lhs.c() * rhs.d()) - (lhs.d() * rhs.c());
    decltype(lhs.a() * rhs.a()) tnj = (lhs.a() * rhs.c()) + (lhs.c() * rhs.a()) + (lhs.d() * rhs.b()) - (lhs.b() * rhs.d());
    decltype(lhs.a() * rhs.a()) tnk = (lhs.a() * rhs.d()) + (lhs.d() * rhs.a()) + (lhs.b() * rhs.c()) - (lhs.c() * rhs.b());
    return {tn, tni, tnj, tnk};
}

/**
 * Mul operator between quaternion and std:complex.
 */
template<typename _tA, typename _tB>
auto operator*(const Quaternion<_tA>& lhs, const std::complex<_tB>& rhs) -> Quaternion<decltype(lhs.a() * rhs.real())> {
    decltype(lhs.a() * rhs.real()) tn = (lhs.a() * rhs.real()) - (lhs.b() * rhs.imag());
    decltype(lhs.a() * rhs.real()) tni = (lhs.a() * rhs.imag()) + (lhs.b() * rhs.real());
    decltype(lhs.a() * rhs.real()) tnj = (lhs.c() * rhs.real()) + (lhs.d() * rhs.imag());
    decltype(lhs.a() * rhs.real()) tnk = (lhs.d() * rhs.real()) - (lhs.c() * rhs.imag());
    return {tn, tni, tnj, tnk};
}

/**
 * Mul operator between std:complex and quaternion.
 */
template<typename _tA, typename _tB>
auto operator*(const std::complex<_tB>& lhs, const Quaternion<_tA>& rhs) -> Quaternion<decltype(lhs.real() * rhs.a())> {
    decltype(lhs.real() * rhs.a()) tn = (lhs.real() * rhs.a()) - (lhs.imag() * rhs.b());
    decltype(lhs.real() * rhs.a()) tni = (lhs.real() * rhs.b()) + (lhs.imag() * rhs.a());
    decltype(lhs.real() * rhs.a()) tnj = (lhs.real() * rhs.c()) - (lhs.imag() * rhs.d());
    decltype(lhs.real() * rhs.a()) tnk = (lhs.real() * rhs.d()) + (lhs.imag() * rhs.c());
    return {tn, tni, tnj, tnk};
}

/**
 * Mul operator between quaternion and scalar.
 */
template<typename _tA, typename _tB>
auto operator*(const Quaternion<_tA>& lhs, const _tB& rhs) -> Quaternion<decltype(lhs.a() * rhs)> {
    return {lhs.a() * rhs, lhs.b() * rhs, lhs.c() * rhs, lhs.d() * rhs};
}

/**
 * Mul operator between scalar and quaternion.
 */
template<typename _tA, typename _tB>
auto operator*(const _tA& lhs, const Quaternion<_tB>& rhs) -> Quaternion<decltype(lhs * rhs.a())> {
    return operator*(rhs, lhs);
}

/**
 * Div operator between two quaternions.
 */
template<typename _tA, typename _tB>
auto operator/(const Quaternion<_tA>& lhs, const Quaternion<_tB>& rhs) -> Quaternion<decltype((lhs.a() * rhs.a()) / rhs.a())> {
    decltype(lhs.a() * rhs.a()) tn  = (lhs.a() * rhs.a()) + (lhs.b() * rhs.b()) + (lhs.c() * rhs.c()) + (lhs.d() * rhs.d());
    decltype(lhs.a() * rhs.a()) tni = - (lhs.a() * rhs.b()) + (lhs.b() * rhs.a()) - (lhs.c() * rhs.d()) + (lhs.d() * rhs.c());
    decltype(lhs.a() * rhs.a()) tnj = - (lhs.a() * rhs.c()) + (lhs.c() * rhs.a()) - (lhs.d() * rhs.b()) + (lhs.b() * rhs.d());
    decltype(lhs.a() * rhs.a()) tnk = - (lhs.a() * rhs.d()) + (lhs.d() * rhs.a()) - (lhs.b() * rhs.c()) + (lhs.c() * rhs.b());
    decltype(rhs.a()) norm = std::norm(rhs);
    return {tn / norm, tni / norm, tnj / norm, tnk / norm};
}

/**
 * Div operator between quaternion and scalar.
 */
template<typename _tA, typename _tB>
auto operator/(const Quaternion<_tA>& lhs, const _tB& rhs) -> Quaternion<decltype(lhs.a() / rhs)> {
    return {lhs.a() / rhs, lhs.b() / rhs, lhs.c() / rhs, lhs.d() / rhs};
}

/**
 * Div operator between scalar and quaternion.
 */
template<typename _tA, typename _tB>
auto operator/(const _tB& lhs, const Quaternion<_tA>& rhs) -> Quaternion<decltype((rhs.a() * lhs) / rhs.a())> {
    decltype(rhs.a()) norm = std::norm(rhs);
    return {(rhs.a() * lhs) / norm, -(rhs.b() * lhs) / norm, -(rhs.c() * lhs) / norm, -(rhs.d() * lhs) / norm};
}

/**
 * Trivial comparison operator between quaternions.
 */
template<typename _tA, typename _tB>
bool operator==(const _tA& lhs, const _tB& rhs) {
    return lhs.a() == rhs.a() && lhs.b() == rhs.b() && lhs.c() == rhs.c() && lhs.d() == rhs.d();
}

/**
 * Stream operator for quaternion.
 */
template<typename T>
std::ostream& operator<< (std::ostream& os, const Quaternion<T>& obj) {
    os << "(" << obj.a() << "," << obj.b() << "," << obj.c() << "," << obj.d() << ")"; 
    return os;
}

/**
 * Normalization function
 */
template<typename T>
auto normalized(const Quaternion<T>& quat) -> Quaternion<decltype(quat.a() / std::abs(quat))> {
    return quat / std::abs(quat);
}

/**
 * Inverse function
 */
template<typename T>
auto inverse(const Quaternion<T>& quat) {
    return std::conj(quat) / std::norm(quat);
}

#endif // QUATER_H
