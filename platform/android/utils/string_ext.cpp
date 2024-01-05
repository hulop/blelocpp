/*******************************************************************************
 * Copyright (c) 2014, 2015  IBM Corporation and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#ifdef ANDROID_STL_EXT

#include "string_ext.hpp"

namespace std {

    // to_string
    template<typename T>
    std::string to_string(const T &n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }

    std::string to_string(const int& n){
        return to_string(n);
    }

    std::string to_string(const long& n){
        return to_string(n);
    }

    std::string to_string(const size_t& n){
        return to_string(n);
    }


    template<class T>
    T sto_val(const std::string &str, std::size_t *idx = 0, int base = 10) {
        std::stringstream ss(str);
        T val;
        ss >> val;
        return val;
    }

    int stoi(const std::string &str, std::size_t *idx, int base) {
        return sto_val<int>(str, idx, base);
    }

    long stol(const std::string &str, std::size_t *idx, int base) {
        return sto_val<long>(str, idx, base);
    }

    unsigned long stoul(const std::string &str, std::size_t *idx, int base) {
        return sto_val<unsigned long>(str, idx, base);
    }

    long long stoll(const std::string &str, std::size_t *idx, int base) {
        return sto_val<long long>(str, idx, base);
    }

    unsigned long long stoull(const std::string &str, std::size_t *idx, int base) {
        return sto_val<unsigned long long>(str, idx, base);
    }

    float stof(const std::string &str, std::size_t *idx) {
        return sto_val<float>(str, idx);
    }

    double stod(const std::string &str, std::size_t *idx) {
        return sto_val<double>(str, idx);
    }

    long double stold(const std::string &str, std::size_t *idx) {
        return sto_val<long double>(str, idx);
    }
}

#endif /* ANDROID_STL_EXT */
