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

#ifndef Building_hpp
#define Building_hpp

#include <stdio.h>
#include <map>
#include "Location.hpp"
#include "FloorMap.hpp"
#ifdef ANDROID_STL_EXT
#include "string_ext.hpp"
#endif /* ANDROID_STL_EXT */

namespace loc {

    class Building{
        int minFloor_;
        int maxFloor_;
        std::map<int, FloorMap> floors;
        
    public:
        using Ptr = std::shared_ptr<Building>;
        
        Building() = default;
        ~Building() = default;
        Building(std::map<int, FloorMap> _floors);
        
        const FloorMap& getFloorAt(int floor_num) const;
        const FloorMap& getFloorAt(const Location& location) const;
        size_t nFloors() const { return this->floors.size(); }

        bool isMovable(const Location& location) const;
        bool isValid(const Location& location) const;
        bool isFloor(const Location& location) const;
        bool isWall(const Location& location) const;
        bool isStair(const Location& location) const;
        bool isElevator(const Location& location) const;
        bool isEscalator(const Location& location) const;
        bool isEscalatorEnd(const Location& location) const;
        bool isEscalatorGroup(const Location& location) const;

        bool checkCrossingWall(const Location& start, const Location& end) const;
        bool checkMovableRoute(const Location& start, const Location& end) const;
        
        int minFloor() const;
        int maxFloor() const;
        bool isValidFloor(int floor);
        
        double estimateWallAngle(const Location& start, const Location &end) const;
    };
    
    class BuildingBuilder{
    private:
        std::map<int, CoordinateSystemParameters> mFloorCoordinateSystemParametersMap;
        std::map<int, std::string> mFloorImagePathMap;
    public:
        BuildingBuilder& addFloorCoordinateSystemParametersAndImagePath(int floor_num, CoordinateSystemParameters coordinateSystemParameters, std::string imagePath);
        Building build();
    };
}
#endif /* MapFloorsModel_hpp */
