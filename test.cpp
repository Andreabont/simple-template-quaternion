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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Quaternion tests"

#include <complex>
#include "Quaternion.h"
#include <boost/test/unit_test.hpp> //VERY IMPORTANT - include this last


/**
 * Comparison operator for test
 */

const double epsilon = 1e-05;

bool operator==(const Quaternion<double>& lhs, const Quaternion<double>& rhs) {
    bool check_a = std::abs(lhs.a() - rhs.a()) <= ((std::abs(lhs.a()) < std::abs(rhs.a()) ? std::abs(rhs.a()) : std::abs(lhs.a())) * epsilon);
    bool check_b = std::abs(lhs.b() - rhs.b()) <= ((std::abs(lhs.b()) < std::abs(rhs.b()) ? std::abs(rhs.b()) : std::abs(lhs.b())) * epsilon);
    bool check_c = std::abs(lhs.c() - rhs.c()) <= ((std::abs(lhs.c()) < std::abs(rhs.c()) ? std::abs(rhs.c()) : std::abs(lhs.c())) * epsilon);
    bool check_d = std::abs(lhs.d() - rhs.d()) <= ((std::abs(lhs.d()) < std::abs(rhs.d()) ? std::abs(rhs.d()) : std::abs(lhs.d())) * epsilon);
    return check_a && check_b && check_c && check_d;
}

bool compare_double(const double& a, const double& b) {
    return std::abs(a - b) <= ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);

}

/** CONSTRUCTION OF A QUATERNION **/

BOOST_AUTO_TEST_CASE(quaternion_costruction_from_components) {
    Quaternion<double> test(0.1,0.5,0.9,1);
    BOOST_CHECK_EQUAL(test.a(), 0.1);
    BOOST_CHECK_EQUAL(test.b(), 0.5);
    BOOST_CHECK_EQUAL(test.c(), 0.9);
    BOOST_CHECK_EQUAL(test.d(), 1);
}

BOOST_AUTO_TEST_CASE(quaternion_costruction_from_complex) {
    std::complex<double> a(0.1,0.5);
    std::complex<double> b(0.9,1);
    Quaternion<double> test(a,b);
    BOOST_CHECK_EQUAL(test.complex_a(), a);
    BOOST_CHECK_EQUAL(test.complex_b(), b);
}

BOOST_AUTO_TEST_CASE(quaternion_costruction_from_copy) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    Quaternion<double> b(a);
    BOOST_CHECK_EQUAL(a, b);
}

/** UNARY OPERATORS **/

BOOST_AUTO_TEST_CASE(quaternion_conjugation) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    Quaternion<double> b(0.1,-0.5,-0.9,-1);
    BOOST_CHECK_EQUAL(std::conj(a), b);
}

BOOST_AUTO_TEST_CASE(quaternion_norm) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    double b = 2.07000;
    BOOST_CHECK_EQUAL(compare_double(std::norm(a), b), true);
}

BOOST_AUTO_TEST_CASE(quaternion_abs) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    double b = 1.43875;
    BOOST_CHECK_EQUAL(compare_double(std::abs(a), b), true);
}

BOOST_AUTO_TEST_CASE(quaternion_normalization) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    Quaternion<double> b(0.0695048,0.347524,0.625543,0.695048);
    BOOST_CHECK_EQUAL(normalized(a), b);
}

BOOST_AUTO_TEST_CASE(quaternion_inversion) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    Quaternion<double> b(0.0483092,-0.241546,-0.434783,-0.483092);
    BOOST_CHECK_EQUAL(inverse(a), b);
}

/** BINARY OPERATOR: SUM **/

BOOST_AUTO_TEST_CASE(sum_between_quaternions) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    Quaternion<double> b(0.9,0.5,0.1,0);
    Quaternion<double> c(1,1,1,1);
    BOOST_CHECK_EQUAL(a + b, c);
}

BOOST_AUTO_TEST_CASE(sum_between_scalar_and_quaternion) {
    double a = 1;
    Quaternion<double> b(0.9,0.5,0.1,0);
    Quaternion<double> c(1.9,0.5,0.1,0);
    BOOST_CHECK_EQUAL(a + b, c);
}

BOOST_AUTO_TEST_CASE(sum_between_quaternion_and_scalar) {
    Quaternion<double> a(0.9,0.5,0.1,0);
    double b = 1;
    Quaternion<double> c(1.9,0.5,0.1,0);
    BOOST_CHECK_EQUAL(a + b, c);
}

BOOST_AUTO_TEST_CASE(sum_between_complex_and_quaternion) {
    std::complex<double> a (1,1);
    Quaternion<double> b(0.9,0.5,0.1,0);
    Quaternion<double> c(1.9,1.5,0.1,0);
    BOOST_CHECK_EQUAL(a + b, c);
}

BOOST_AUTO_TEST_CASE(sum_between_quaternion_and_complex) {
    Quaternion<double> a(0.9,0.5,0.1,0);
    std::complex<double> b (1,1);
    Quaternion<double> c(1.9,1.5,0.1,0);
    BOOST_CHECK_EQUAL(a + b, c);
}

/** BINARY OPERATOR: DIFFERENCE **/

BOOST_AUTO_TEST_CASE(difference_between_quaternions) {
    Quaternion<double> a(1,1,1,1);
    Quaternion<double> b(0.1,0.5,0.9,1);
    Quaternion<double> c(0.9,0.5,0.1,0);
    BOOST_CHECK_EQUAL(a - b, c);
}

BOOST_AUTO_TEST_CASE(difference_between_scalar_and_quaternion) {
    double a = 1;
    Quaternion<double> b(0.9,0.5,0.1,0);
    Quaternion<double> c(0.1,-0.5,-0.1,0);
    BOOST_CHECK_EQUAL(a - b, c);
}

BOOST_AUTO_TEST_CASE(difference_between_quaternion_and_scalar) {
    Quaternion<double> a(0.9,0.5,0.1,0);
    double b = 1;
    Quaternion<double> c(-0.1,0.5,0.1,0);
    BOOST_CHECK_EQUAL(a - b, c);
}

BOOST_AUTO_TEST_CASE(difference_between_complex_and_quaternion) {
    std::complex<double> a (1,1);
    Quaternion<double> b(0.9,0.5,0.1,0);
    Quaternion<double> c(0.1,0.5,-0.1,0);
    BOOST_CHECK_EQUAL(a - b, c);
}

BOOST_AUTO_TEST_CASE(difference_between_quaternion_and_complex) {
    Quaternion<double> a(0.9,0.5,0.1,0);
    std::complex<double> b (1,1);
    Quaternion<double> c(-0.1,-0.5,0.1,0);
    BOOST_CHECK_EQUAL(a - b, c);
}

/** BINARY OPERATOR: MULTIPLICATION **/

BOOST_AUTO_TEST_CASE(multiplication_between_quaternions) {
    Quaternion<double> a(-1,1,-1,1);
    Quaternion<double> b(0.1,0.5,0.9,1);
    Quaternion<double> c(-0.7,-2.3,-1.5,0.5);
    BOOST_CHECK_EQUAL(a * b, c);
}

BOOST_AUTO_TEST_CASE(multiplication_between_scalar_and_quaternion) {
    double a = 2;
    Quaternion<double> b(0.1,0.5,0.9,1);
    Quaternion<double> c(0.2,1,1.8,2);
    BOOST_CHECK_EQUAL(a * b, c);
}

BOOST_AUTO_TEST_CASE(multiplication_between_quaternion_and_scalar) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    double b = 2;
    Quaternion<double> c(0.2,1,1.8,2);
    BOOST_CHECK_EQUAL(a * b, c);
}

BOOST_AUTO_TEST_CASE(multiplication_between_complex_and_quaternion) {
    std::complex<double> a (1,1);
    Quaternion<double> b(0.1,0.5,0.9,1);
    Quaternion<double> c(-0.4,0.6,-0.1,1.9);
    BOOST_CHECK_EQUAL(a * b, c);
}

BOOST_AUTO_TEST_CASE(multiplication_between_quaternion_and_complex) {
    Quaternion<double> a(0.1,0.5,0.9,1);
    std::complex<double> b (1,1);
    Quaternion<double> c(-0.4,0.6,1.9,0.1);
    BOOST_CHECK_EQUAL(a * b, c);
}

/** BINARY OPERATOR: DIVISION **/

BOOST_AUTO_TEST_CASE(division_between_quaternions) {
    Quaternion<double> a(-1,1,-1,1);
    Quaternion<double> b(0.1,0.5,0.9,1);
    Quaternion<double> c(0.241546,1.207729,0.628019,-0.144928);
    BOOST_CHECK_EQUAL(a / b, c);
}

BOOST_AUTO_TEST_CASE(division_between_scalar_and_quaternion) {
    double a = 2;
    Quaternion<double> b(0.1,0.5,0.9,1);
    Quaternion<double> c(0.0966184,-0.483092,-0.869565,-0.966184);
    BOOST_CHECK_EQUAL(a / b, c);
}

BOOST_AUTO_TEST_CASE(division_between_quaternion_and_scalar) {
    Quaternion<double> a(-1,1,-1,1);
    double b = 2;
    Quaternion<double> c(-0.5,0.5,-0.5,0.5);
    BOOST_CHECK_EQUAL(a / b, c);
}
