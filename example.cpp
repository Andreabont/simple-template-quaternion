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

#include <iostream>
#include "Quaternion.h"

int main(int argc, char **argv) {    

    Quaternion<int> a(1,0,1,0);
    Quaternion<double> b(1,0.5,0.5,0.75);
    Quaternion<float> c(3,0.5,1.5,0.75);
        
    std::complex<double> ca(1,2);
    std::complex<double> cb(3,4);
        
    std::cout << "Real part of " << a << " is " << a.real() << std::endl;
    
    std::cout << "Unreal part of " << b << " is " << b.unreal() << std::endl;
    
    std::cout << "Component of " << a << " is " << a.a() << ", " << a.b() << ", " << a.c() << ", " << a.d() << std::endl;
    
    std::cout << "Norm of " << a << " is " << std::norm(a) << std::endl;
        
    std::cout << "Modulus of " << c << " is " << std::abs(c) << std::endl;

    std::cout << "Conjugate of " << b << " is " << std::conj(b) << std::endl;
    
    std::cout << "Normalization of " << b << " is " << normalized(b) << std::endl;
        
    std::cout << "Inverse of " << b << " is " << inverse(b) <<  std::endl;
    
    std::cout << a << " + " << b << " = " << a + b << std::endl;
        
    std::cout << a << " + " << 3 << " = " << a + 3 << std::endl;
        
    std::cout << b << " + complex " << ca << " = " << b + ca << std::endl;
    
    std::cout << a << " - " << b << " = " << a - b << std::endl;
    
    std::cout << a << " * " << b << " = " << a * b << std::endl;

    std::cout << a << " * " << 2.2 << " = " << a * 2.2 << std::endl;
    
    std::cout << a << " / " << b << " = " << a / b << std::endl;

    std::cout << a << " / " << 2.2 << " = " << a / 2.2 << std::endl;
        
    std::cout << 2.2 << " / " << b << " = " << 2.2 / b << std::endl;
    
    Quaternion<double> component (ca,cb);
    
    std::cout << "Construct quaternion from complex " << ca << " and " << cb << " = " << component <<  std::endl;
    
    std::cout << "Quaternion " << component << " has complex component " << component.complex_a() << " and " << component.complex_b() <<  std::endl;
    
    std::cout << a << " is NaN? " << std::boolalpha << std::isnan(a) << std::endl;
    
    std::cout << a << " is infinite? " << std::boolalpha << std::isinf(a) << std::endl;

    std::cout << a << " is finite? " << std::boolalpha << std::isfinite(a) << std::endl;

    return 0;
    
}
