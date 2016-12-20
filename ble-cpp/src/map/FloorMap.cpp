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

#include "FloorMap.hpp"
#include <cmath>

namespace loc{

    namespace color{
        const Color colorFloor = white;
        const Color colorWall = black;
        const Color colorStairs = blue;
        const Color colorElevator = yellow;
        const Color colorEscalator = lime;
        
        const Color colorEscalatorEnd = green;
    }
    
    using namespace color;
    
    const std::vector<Color> colorTransitionArea{colorStairs, colorElevator, colorEscalator};
        
    FloorMap::FloorMap(ImageHolder image, CoordinateSystem coordSys){
        mImage = image;
        mCoordSys = coordSys;
        
        for(const Color&c : colorTransitionArea){
            mImage.setUpIndexForColor(c);
        }
    }

    Color FloorMap::getColor(const Location& location) const{
        if(isInsideFloor(location)){
            Location localCoord = mCoordSys.worldToLocalState(location);
            int x = getX(localCoord);
            int y = getY(localCoord);
            Color pixelColor = mImage.get(y, x);
            return pixelColor;
        }else{
            return color::colorFloor;
        }
    }
    
    int FloorMap::getX(const loc::Location &location) const{
        int x = doubleToImageCoordinate(location.x());
        return x;
    }
    
    int FloorMap::getY(const loc::Location &location) const{
        int y = doubleToImageCoordinate(location.y());
        return y;
    }
    
    ImageHolder::Point FloorMap::getPoint(const loc::Location &location) const{
        Location localCoord = mCoordSys.worldToLocalState(location);
        int x = getX(localCoord);
        int y = getY(localCoord);
        ImageHolder::Point p(x,y);
        return p;
    }
    
    int FloorMap::doubleToImageCoordinate(double x) const{
        int i = static_cast<int>(std::round(x));
        return i;
    }
        
    bool FloorMap::isMovable(const Location& location) const{
        if(isWall(location)){
            return false;
        }
        return true;
    }

    bool FloorMap::isFloor(const Location &location) const{
        return checkColor(location, color::colorFloor);
    }
    
    bool FloorMap::isValid(const Location& location) const{
        return true;
    }
    
    bool FloorMap::isInsideFloor(const Location& location) const{
        Location localCoord = mCoordSys.worldToLocalState(location);
        int x = getX(localCoord);
        int y = getY(localCoord);
        
        int cols = mImage.cols();
        int rows = mImage.rows();
        
        if ( 0<=x && x<cols && 0<=y && y<rows ){
            return true;
        }else{
            return false;
        }
    }
    

    bool FloorMap::checkColor(const Location& location, const Color& color) const{
        Color pixelColor = getColor(location);
        return pixelColor.equals(color);
    }

    bool FloorMap::isWall(const Location& location) const{
        return checkColor(location, color::colorWall);
    }

    bool FloorMap::isStairs(const Location& location) const {
        return checkColor(location, color::colorStairs);
    }

    bool FloorMap::isElevator(const Location& location) const{
        return checkColor(location, color::colorElevator);
    }

    bool FloorMap::isEscalator(const Location& location) const{
        return checkColor(location, color::colorEscalator);
    }
    
    bool FloorMap::isEscalatorEnd(const Location& location) const{
        return checkColor(location, color::colorEscalatorEnd);
    }

    double FloorMap::wallCrossingRatio(const Location& start, const Location& end) const{
        Location startLocal = mCoordSys.worldToLocalState(start);
        double x0 = (startLocal.x());
        double y0 = (startLocal.y());

        Location endLocal = mCoordSys.worldToLocalState(end);
        double x1 = (endLocal.x());
        double y1 = (endLocal.y());

        double norm = sqrt(pow(x1-x0,2)+pow(y1-y0,2));

        int norm_int = static_cast<int>(norm) + 1;

        double dx = (x1-x0)/norm_int;
        double dy = (y1-y0)/norm_int;

        double x = x0;
        double y = y0;
        
        bool startIsEscEnd = isEscalatorEnd(start);

        int count=0;
        while(count<=norm_int){
            int yInt = doubleToImageCoordinate(y);
            int xInt = doubleToImageCoordinate(x);
            Color c = color::colorFloor;
            bool pixelIsValid = mImage.checkValid(yInt, xInt);
            if(pixelIsValid){
                c = mImage.get(yInt, xInt);
            }else{
                c = color::colorFloor;
            }
            if(c.equals(color::colorWall)){
                return ((double)count-1)/norm_int;
            }
            if(startIsEscEnd && c.equals(color::colorEscalator)){
                return ((double)count-1)/norm_int;
            }
            x+=dx;
            y+=dy;
            count++;
        }
        return ((double)count-1)/norm_int;
    }

    bool FloorMap::checkCrossingWall(const Location& start, const Location& end) const {
        double ratio = wallCrossingRatio(start, end);
        return ratio < 1.0;
    }
    
    double FloorMap::estimateWallAngle(const Location &start, const Location& end) const{
        
        double r = wallCrossingRatio(start, end);
        double dx = end.x() - start.x();
        double dy = end.y() - start.y();
        double x = start.x() + dx * r;
        double y = start.y() + dy * r;
        double norm = sqrt(dx*dx+dy*dy);
        double a = std::atan2(dy, dx);
        
        Location nearWall(start);
        Location nearWall2(end);
        nearWall.x(x);
        nearWall.y(y);
        
        int sign = 1;
        for(double angle = 1; angle<=90; angle++){
            for(int i=0; i<2; i++){
                sign *= -1;
                double newAngle = a + sign*(angle/180*M_PI);
                double x2 = x + std::cos(newAngle) * norm;
                double y2 = y + std::sin(newAngle) * norm;
                nearWall2.x(x2);
                nearWall2.y(y2);
                if(wallCrossingRatio(nearWall, nearWall2) >= 1.0){
                    newAngle = Pose::normalizeOrientaion(newAngle);// Returns normalized angle.
                    if(newAngle < -M_PI || M_PI < newAngle){
                        BOOST_THROW_EXCEPTION(LocException("estimated wall angle is out of range."));
                    }
                    return newAngle;
                }
            }
        }
        return std::numeric_limits<float>::quiet_NaN();;
    }
    
    const CoordinateSystem& FloorMap::coordinateSystem() const{
        return mCoordSys;
    }
    
    bool FloorMap::isTransitionArea(const Location& location) const{
        if(isEscalator(location) || isElevator(location) || isStairs(location)){
            return true;
        }else{
            return false;
        }
    }
    
    std::vector<Location> FloorMap::findClosestTransitionAreaLocations(const Location& location) const{
        Location localCoord = mCoordSys.worldToLocalState(location);
        ImageHolder::Point pIm = getPoint(location);
        ImageHolder::Point pClosest;
        
        double dmin = std::numeric_limits<double>::max();
        for(auto& c: colorTransitionArea){
            auto psRet = mImage.findClosestPoints(c, pIm);
            if(psRet.size()==0) continue;
        
            auto pRet = psRet.at(0);
            double dist = ImageHolder::Point::distance(pIm, pRet);
            if(dist < dmin){
                dmin = dist;
                pClosest = pRet;
            }
        }
        std::vector<Location> locsRet;
        if(std::numeric_limits<double>::max()<=dmin){
            return locsRet;
        }
        
        localCoord.x(pClosest.x);
        localCoord.y(pClosest.y);
        
        Location locRet = mCoordSys.localToWorldState(localCoord);
        locsRet.push_back(locRet);
        return locsRet;
    }
}
