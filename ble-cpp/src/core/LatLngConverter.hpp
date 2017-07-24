/*******************************************************************************
 * Copyright (c) 2014, 2016  IBM Corporation and others
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

#ifndef LatLngConverter_hpp
#define LatLngConverter_hpp

#include <stdio.h>
#include <string>
#include <sstream>
#include <memory>
#include <iomanip>
#include "Location.hpp"
#include "LatLngUtil.hpp"
#include "Heading.hpp"

namespace loc {
    
    template <class Tstate>
    class GlobalState: public Tstate{
    private:
        double lat_;
        double lng_;
    public:
        using Tstate::toString;
        
        GlobalState(const Tstate& s): Tstate(s){}
        
        double lat() const{
            return lat_;
        }
        double lng() const{
            return lng_;
        }
        void lat(double lat){
            lat_ = lat;
        }
        void lng(double lng){
            lng_ = lng;
        }
        
        std::string toString() const{
            std::stringstream stream;
            stream << Tstate::toString() << ",";
            stream << std::fixed << std::setprecision(8) << lat() << "," << lng() ;
            std::string str = stream.str();
            return str;
        }
    };
    
    // for string stream
    template <class Tstate>
    std::ostream& operator<<(std::ostream&os, const GlobalState<Tstate>& location){
        os << location.toString();
        return os;
    }
    
    class LatLngConverter{
    private:
        Anchor anchor_;
    public:
        using Ptr = std::shared_ptr<LatLngConverter>;
        
        LatLngConverter(const Anchor& anchor);
        void anchor(const Anchor& anchor);
        Anchor anchor() const;
        
        template<class Tstate>
        GlobalState<Tstate> localToGlobal(const Tstate& local);
        template<class Tstate>
        Tstate globalToLocal(const GlobalState<Tstate>& global);
        
        // convert degree in the global coordinate to radian in the local coordinate
        double headingGlobalToLocal(double heading);
        LocalHeading headingGlobalToLocal(const Heading&);
    };
}
#endif /* LatLngConverter_hpp */
