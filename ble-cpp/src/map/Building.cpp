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

#include "Building.hpp"
#include "LocException.hpp"
#include <cmath>

#include <boost/tuple/tuple_io.hpp>
#include <boost/algorithm/minmax_element.hpp>

namespace loc{

    Building::Building(std::map<int, FloorMap> _floors){
        
        floors = _floors;
        
        size_t size = _floors.size();
        int floors[size];
        int i=0;
        for(auto iter=_floors.begin(); iter!=_floors.end(); iter++){
            floors[i] = iter->first;
            i++;
        }
        std::pair<int*,int*> twin_data = boost::minmax_element( &floors[0], &floors[size] );
        
        minFloor_ = *twin_data.first;
        maxFloor_ = *twin_data.second;
    }
    
    const FloorMap& Building::getFloorAt(int floor_num) const{
        if(this->nFloors() <= 0){
            BOOST_THROW_EXCEPTION(LocException("The number of floors is less than 1."));
        }
        if( floor_num < minFloor()){
            BOOST_THROW_EXCEPTION(LocException("floor_num < minFloor"));
        }
        if( maxFloor() < floor_num){
            BOOST_THROW_EXCEPTION(LocException("maxFloor < floor_num"));
        }
        return floors.at(floor_num);
    }
    
    const FloorMap& Building::getFloorAt(const Location& location) const{
        int floor_int = static_cast<int>(location.floor());
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap;
    }

    bool Building::isMovable(const Location& location) const{
        int floor_int = static_cast<int>(location.floor());
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.isMovable(location);
    }

    bool Building::isValid(const Location& location) const{
        int floor_int = static_cast<int>(location.floor());
        if(floors.count(floor_int)==0){
            return false;
        }
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.isValid(location);
    }

    bool Building::isFloor(const Location& location) const{
        int floor_int = static_cast<int>(location.floor());
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.isFloor(location);
    }
    
    bool Building::isWall(const Location& location) const{
        int floor_int = static_cast<int>(location.floor());
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.isWall(location);
    }

    bool Building::isStair(const Location& location) const{
        int floor_int = static_cast<int>(location.floor());
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.isStairs(location);
    }

    bool Building::isElevator(const Location& location) const{
        int floor_int = static_cast<int>(location.floor());
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.isElevator(location);
    }
    
    bool Building::isEscalator(const Location& location) const{
        const FloorMap& floorMap = getFloorAt(location);
        return floorMap.isEscalator(location);
    }

    bool Building::isEscalatorEnd(const Location& location) const{
        const FloorMap& floorMap = getFloorAt(location);
        bool isEscEnd = floorMap.isEscalatorEnd(location);
        return isEscEnd;
    }
    
    bool Building::isEscalatorGroup(const Location& location) const{
        if(isEscalator(location)){
            return true;
        }
        if(isEscalatorEnd(location)){
            return true;
        }
        return false;
    }
    
    bool Building::checkCrossingWall(const Location& start, const Location& end) const{
        double floor0 = start.floor();
        double floor1 = end.floor();

        if(floor0!=floor1){
            return true;
        }

        int floor_int = static_cast<int>(floor0);
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.checkCrossingWall(start, end);
    }
    
    bool Building::checkMovableRoute(const Location& start, const Location& end) const{
        if(! isMovable(end)){
            return false;
        }
        // Do not allow to move from escalater_end to escalator
        if(isEscalatorEnd(start)){
            if(isEscalator(end)){
                return false;
            }
        }
        if(checkCrossingWall(start, end)){
            return false;
        }
        return true;
    }
    
    int Building::minFloor() const{
        return minFloor_;
    }
    
    int Building::maxFloor() const{
        return maxFloor_;
    }

    bool Building::isValidFloor(int floor){
        if(minFloor()<=floor
           && floor<=maxFloor()){
            return true;
        }else{
            return false;
        }
    }
    
    double Building::estimateWallAngle(const Location& start, const Location& end) const{
        double floor0 = start.floor();
        double floor1 = end.floor();
        if(floor0 != floor1){
            BOOST_THROW_EXCEPTION(LocException("start_location.floor != end_location.floor"));
        }
        int floor_int = static_cast<int>(floor0);
        const FloorMap& floorMap = getFloorAt(floor_int);
        return floorMap.estimateWallAngle(start, end);
    }
    
    /**
     Implementation of building builder
     **/
     
    BuildingBuilder& BuildingBuilder::addFloorCoordinateSystemParametersAndImagePath(int floor_num, CoordinateSystemParameters coordinateSystemParameters, std::string imagePath){
        
        mFloorCoordinateSystemParametersMap[floor_num] = coordinateSystemParameters;
        mFloorImagePathMap[floor_num] = imagePath;
        return *this;
    }
    
    Building BuildingBuilder::build(){
        std::map<int, FloorMap> floorsMap;
        for(auto iter= mFloorCoordinateSystemParametersMap.begin();
            iter!=mFloorCoordinateSystemParametersMap.end();
            iter++){
            
            int floor_num = iter->first;
            std::string name = std::to_string(floor_num);
            CoordinateSystemParameters coordSysParams = mFloorCoordinateSystemParametersMap[floor_num];
            std::string path = mFloorImagePathMap[floor_num];
            
            ImageHolder image(path,name);
            CoordinateSystem coordSys(coordSysParams);
            FloorMap floorMap(image, coordSys);
            floorsMap[floor_num] = floorMap;
        }
        
        Building building(floorsMap);
        return building;
    }
    
}
