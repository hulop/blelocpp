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

#ifndef FloorMap_hpp
#define FloorMap_hpp

#include <stdio.h>

#include "CoordinateSystem.hpp"
#include "ImageHolder.hpp"
#include "State.hpp"

namespace loc{
    class FloorMap{
    protected:
        CoordinateSystem mCoordSys;
        ImageHolder mImage;

        Color getColor(const Location& location) const;
        bool checkColor(const Location& location, const Color& color) const;
        int getX(const Location& location) const;
        int getY(const Location& location) const;
        ImageHolder::Point getPoint(const Location& location) const;
        int doubleToImageCoordinate(double x) const;

    public:
        FloorMap() = default;
        ~FloorMap() = default;
        FloorMap(ImageHolder image, CoordinateSystem coordSys);

        bool isMovable(const Location& location) const;
        bool isValid(const Location& location) const;
        bool isFloor(const Location& location) const;
        bool isWall(const Location& location) const;
        bool isStairs(const Location& location) const;
        bool isElevator(const Location& location) const;
        bool isEscalator(const Location& location) const;
        bool isEscalatorEnd(const Location& location) const;

        bool checkMovable(const Location& start, const Location& end) const;

        double wallCrossingRatio(const Location& start, const Location& end) const;
        bool checkCrossingWall(const Location& start, const Location& end) const;

        double estimateWallAngle(const Location&start, const Location& end) const;
        
        const CoordinateSystem& coordinateSystem() const;
        
        bool isTransitionArea(const Location& location) const;
        std::vector<Location> findClosestTransitionAreaLocations(const Location& location) const;
        
        template<class Archive>
        void serialize(Archive & ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(mCoordSys));
            ar(CEREAL_NVP(mImage));
        }
        
    protected:
        bool isInsideFloor(const Location& location) const;
    };
}

CEREAL_CLASS_VERSION(loc::FloorMap, 0);
#endif /* Floor_hpp */
