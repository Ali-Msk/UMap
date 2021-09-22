/* 
 * Copyright 2021 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <set>
#include <algorithm>
#include <map>
#include <unordered_map> 
#include <limits>

//included header file for helper functions
#include "m1_internal.h"

//declare global variables
std:: vector<std:: vector<StreetSegmentIdx>> streetSegmentOfIntersection;
std::unordered_map <StreetIdx, std::vector<IntersectionIdx>> intersectionIdsOfStreets;
std::map <std::string,std:: vector<StreetIdx>> partialNames;
std::map <StreetIdx, double> streetLengths;
std:: vector<double> streetSegmentTravelTime;
std:: vector<double> streetSegmentLength;

bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

    load_successful = loadStreetsDatabaseBIN(map_streets_database_filename); //loading the map succeeded or failed
    
    //exit if map not loaded successful
    if(!load_successful){
        return load_successful;
    }
    
    //load a vector of street segments of intersection
    for(int i = 0; i<getNumIntersections(); i++){
        std :: vector<StreetSegmentIdx> temp;
        for(int j = 0; j<getNumIntersectionStreetSegment(i); j++){
            temp.push_back(getIntersectionStreetSegment(i,j));
        }
        streetSegmentOfIntersection.push_back(temp);
    }
    
    
    //load an undorder map with sets to avoid duplicate intersection names
    std::unordered_map <StreetIdx, std::set<IntersectionIdx>> streetSegmentsOfStreetIds;
    for(int i = 0; i<getNumStreetSegments(); i++){
        StreetSegmentInfo tempInfo = getStreetSegmentInfo(i);
        //if street already already exits in map, add to the set
        if(streetSegmentsOfStreetIds.count(tempInfo.streetID) == 1){
	    streetSegmentsOfStreetIds[tempInfo.streetID].insert(tempInfo.from);
            streetSegmentsOfStreetIds[tempInfo.streetID].insert(tempInfo.to);
	}else{
            //if street doesn't exist create new set and add
	    std::set<StreetSegmentIdx> newSet;
	    streetSegmentsOfStreetIds[tempInfo.streetID] = newSet;
	    streetSegmentsOfStreetIds[tempInfo.streetID].insert(tempInfo.from);
            streetSegmentsOfStreetIds[tempInfo.streetID].insert(tempInfo.to);
	}
    }
    
    
    //convert the previous set in map into a vector and load into intersectoinIdsOfStreet
    for(int i = 0; i<getNumStreets(); i++){
        std:: vector<IntersectionIdx> temp;
        for(int j : streetSegmentsOfStreetIds[i]){
            temp.push_back(j);
        }
        intersectionIdsOfStreets[i]= temp;
    } 
    
    //data structure that stores street ids associated with specific substrings 
    for(int i = 0; i<getNumStreets(); i++){
	std::string name = getStreetName(i);
        
        //removing spaces and converting to lowercase
        std::transform(name.begin(), name.end(), name.begin(), ::tolower); 
        name.erase(std::remove_if(name.begin(), name.end(), isspace), name.end());
        
        //creating a substring for every possible prefix
	for (int j = 1; j <= name.size(); j ++){
            
            std::string substring = name.substr(0, j);
            
            //checking if key exists to avoid duplicates
	    if(partialNames.count(substring) == 1){
                //checking if street id has already been inserted to avoid duplicates
                if(!(std::count(partialNames[substring].begin(), partialNames[substring].end(), i))){
                    partialNames[substring].push_back(i);
                }
                //creating new key if it doesn't exist
	    }else{
	        std::vector<StreetIdx> newVector;
	        partialNames[substring] = newVector;
	        partialNames[substring].push_back(i);
	    }
	}
    }
    
    
    //2 data structures are filled here, one that stores street lengths and the other stores street segments travel times
    for(int i = 0; i<getNumStreetSegments(); i++){        
        StreetSegmentInfo tempInfo;
        tempInfo= getStreetSegmentInfo(i);
            
        //calculate time based on street segment length
        double time = findStreetSegmentLength(i)/tempInfo.speedLimit;
        streetSegmentTravelTime.push_back(time);        
        
        //check if street segment belongs to street
        if(streetLengths.count(tempInfo.streetID) == 1){
            //add street segment length to total length 
            streetLengths[tempInfo.streetID]+=findStreetSegmentLength(i);
        } 
        else{
            //create key and data
            streetLengths[tempInfo.streetID]= findStreetSegmentLength(i);
        }
    }
                         
    return load_successful;
}



void closeMap() {
    //clear data structures
    streetSegmentOfIntersection.clear();
    streetSegmentOfIntersection.shrink_to_fit();
    streetLengths.clear();
    
    std::map<std::string,std:: vector<StreetIdx>>::iterator it;
    for (it=partialNames.begin(); it!=partialNames.end(); it++){
        it->second.clear();
        it->second.shrink_to_fit();   
    }
    
    std::unordered_map<StreetIdx, std::vector<IntersectionIdx>>::iterator it2;
    for (it2=intersectionIdsOfStreets.begin(); it2!=intersectionIdsOfStreets.end(); it2++){
        it2->second.clear();
        it2->second.shrink_to_fit();   
    }
    partialNames.clear();
    intersectionIdsOfStreets.clear();
    
    streetSegmentLength.clear();
    streetSegmentLength.shrink_to_fit();
     
    closeStreetDatabase();
}
                                                                                
double findDistanceBetweenTwoPoints(std:: pair<LatLon,LatLon> points){
    LatLon p1 = points.first;
    LatLon p2 = points.second;
    
    //converting to radians and using pythagorean theorem to solve 
    double latAvg = (p1.latitude() * kDegreeToRadian + p2.latitude() * kDegreeToRadian)/2.0;
    double x1 = p1.longitude() * kDegreeToRadian *  cos(latAvg);
    double y1 = p1.latitude() * kDegreeToRadian;
    double x2 = p2.longitude() * kDegreeToRadian * cos(latAvg);
    double y2 = p2.latitude() * kDegreeToRadian;
 
    //formula for calcuating distance
    double distance = kEarthRadiusInMeters * (sqrt(pow((y2-y1),2) + pow(x2-x1,2)));
    
    return distance;
}

double findStreetSegmentLength(StreetSegmentIdx street_segment_id){
   //get segment info
    StreetSegmentInfo segmentInfo = getStreetSegmentInfo(street_segment_id);
    
    //get segment intersections
    LatLon from = getIntersectionPosition(segmentInfo.from);
    LatLon to = getIntersectionPosition(segmentInfo.to);
    double length = 0;
    
    //check if no curve points
    if(segmentInfo.numCurvePoints == 0){
        length = findDistanceBetweenTwoPoints(std::make_pair(from, to));
        return length;
    }
    
    //first intersection to first curve point 
    length += findDistanceBetweenTwoPoints(std::make_pair(from, getStreetSegmentCurvePoint(street_segment_id, 0)));
    
    //middle curve points 
    for (int  i = 0; i < segmentInfo.numCurvePoints -1; i++){
        from = getStreetSegmentCurvePoint(street_segment_id, i);
        to = getStreetSegmentCurvePoint(street_segment_id, i+1);  
        length += findDistanceBetweenTwoPoints(std::make_pair(from, to));
    }
       
    //last curve point to ending intersection
    from = getStreetSegmentCurvePoint(street_segment_id, segmentInfo.numCurvePoints -1);
    to = getIntersectionPosition(segmentInfo.to);
    length += findDistanceBetweenTwoPoints(std::make_pair(from, to)); 
    
    
    return length;
}

double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    return streetSegmentTravelTime[street_segment_id];
}


IntersectionIdx findClosestIntersection(LatLon my_position){       
    double minDistance = std::numeric_limits<int>::max();
    IntersectionIdx nearestIntersection = 0;
    
    //loop through all intersection and compare distance while storing the minimum
    for(int i = 0; i<getNumIntersections(); i++){
        double tempDistance = findDistanceBetweenTwoPoints(std::make_pair(my_position, getIntersectionPosition(i)));
        if(tempDistance < minDistance){
            minDistance = tempDistance;
            nearestIntersection = i;
        }
    }
    
    return nearestIntersection;
}

std:: vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id){
    return streetSegmentOfIntersection[intersection_id];
}

std:: vector<std:: string> findStreetNamesOfIntersection(IntersectionIdx intersection_id){
   std:: vector<std:: string> namesOfIntersection;
   std:: vector<StreetSegmentIdx> listOfstreeSegments = findStreetSegmentsOfIntersection(intersection_id);
   
   //loop through street segments and get street name from the according matching intersction
   for(int i = 0; i < listOfstreeSegments.size(); i++){
       StreetSegmentInfo info = getStreetSegmentInfo(listOfstreeSegments[i]);
       StreetIdx segmentStreetID = info.streetID;
       std:: string name = getStreetName(segmentStreetID);
       namesOfIntersection.push_back(name);
   }
   
   return namesOfIntersection;
}

std:: vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id){
    std:: vector<IntersectionIdx> adjacentIntersectionList;
    std:: set<IntersectionIdx> adjacentIntersectionsSet;      //place data initially in a set to avoid duplicates
    std:: vector<StreetSegmentIdx> listOfstreeSegments = findStreetSegmentsOfIntersection(intersection_id);
    
    //account for three cases and loop through street segments and analyze each street info
    for(int i = 0; i<listOfstreeSegments.size(); i++){
        StreetSegmentInfo info = getStreetSegmentInfo(listOfstreeSegments[i]);
        //if not one way, adjacent can only be to or from depending on the current id
        if(!info.oneWay){
            if(info.from == intersection_id){
                adjacentIntersectionsSet.insert(info.to);
            }else{
                adjacentIntersectionsSet.insert(info.from);
            }
        }else{
            if(info.from == intersection_id){
                adjacentIntersectionsSet.insert(info.to);
            }
        }
    }
    
    //transfer data from set to vector and return
    for(int i : adjacentIntersectionsSet){
        adjacentIntersectionList.push_back(i);
    }
    
    return adjacentIntersectionList;
}

std:: vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
    return intersectionIdsOfStreets[street_id];
}

std:: vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids){    
    std:: vector<IntersectionIdx> intersections;

    //get intersections of each street 
    std:: vector<IntersectionIdx> firstIntersections = findIntersectionsOfStreet(street_ids.first);
    std:: vector<IntersectionIdx> secondIntersections = findIntersectionsOfStreet(street_ids.second);

    //use set for logarithmic insert and lookup times.
    std:: set<IntersectionIdx> set;
    
    //add all intersections into a set
    for (int i = 0; i<firstIntersections.size();i++){
        set.insert(firstIntersections[i]);
    }
    
    //see if any common ones exist
    for (int i = 0; i<secondIntersections.size();i++){
        if(set.count(secondIntersections[i]) == 1){           
            intersections.push_back(secondIntersections[i]);
        }
    }
    
    return intersections;
}

std:: vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix){
    std:: vector<StreetIdx> streetMatches;
//  
    //convert to lowercase and remove spaces
    std::transform(street_prefix.begin(), street_prefix.end(), street_prefix.begin(), ::tolower); 
    street_prefix.erase(std::remove_if(street_prefix.begin(), street_prefix.end(), isspace), street_prefix.end());
    
    //checking data structure for all prefix associated street ids
    if(partialNames.count(street_prefix) == 1){
        streetMatches=partialNames[street_prefix];
    }
        
    return streetMatches;
}

double findStreetLength(StreetIdx street_id){
    //return precomputed street length from data structure after checking if key exists
    if(streetLengths.count(street_id) == 1){
       return streetLengths[street_id]; 
    }
    return 0;
}

LatLonBounds findStreetBoundingBox(StreetIdx street_id){
    LatLonBounds box;
    LatLon pos_to, pos_from;
   
    double minx=std::numeric_limits<int>::max(), miny=std::numeric_limits<int>::max(), maxx=std::numeric_limits<int>::min(), maxy=std::numeric_limits<int>::min();
    
    //First iterate through street segments, get intersection and curve points positions.
    LatLon curvePointPosition;
    StreetSegmentInfo tempInfo;
    
    for(int i=0;i<getNumStreetSegments();i++){
        tempInfo=getStreetSegmentInfo(i);
            
        if(tempInfo.streetID==street_id){
            //check if intersection positions are min or max
            pos_to=getIntersectionPosition(tempInfo.to);
            pos_from=getIntersectionPosition(tempInfo.from);
            
            //helper functions to check if min or max
            minx=checkMin(pos_to.latitude(), minx);
            miny=checkMin(pos_to.longitude(), miny);
            maxx=checkMax(pos_to.latitude(), maxx);
            maxy=checkMax(pos_to.longitude(), maxy);
            
            minx=checkMin(pos_from.latitude(), minx);
            miny=checkMin(pos_from.longitude(), miny);
            maxx=checkMax(pos_from.latitude(), maxx);
            maxy=checkMax(pos_from.longitude(), maxy);
          
            //now check curve points
            for (int k=0;k<tempInfo.numCurvePoints;k++){
                curvePointPosition=getStreetSegmentCurvePoint(i,k);
                
                minx=checkMin(curvePointPosition.latitude(), minx);
                miny=checkMin(curvePointPosition.longitude(), miny);
                maxx=checkMax(curvePointPosition.latitude(), maxx);
                maxy=checkMax(curvePointPosition.longitude(), maxy);
               
            }           
        }
    }
    //store results in box
    LatLon min(minx,miny);
    LatLon max(maxx,maxy);
    box.min=min;
    box.max=max;
    
    return box;
}

POIIdx findClosestPOI(LatLon my_position, std::string POIname){
    double minDistance =  std::numeric_limits<int>::max();
    double currentDistance=0;
    
    POIIdx closestPOI=0;
    int numPOI=getNumPointsOfInterest();
    //loop through all intersection and compare distance and name while storing the minimum
    for(int i = 0; i<numPOI; i++){
        currentDistance = findDistanceBetweenTwoPoints(std::make_pair(my_position, getPOIPosition(i)));
        if((currentDistance < minDistance)&&(getPOIName(i)==POIname)){
            minDistance = currentDistance;
            closestPOI = i;
        }
    }
    return closestPOI;

}

double findFeatureArea(FeatureIdx feature_id){

        //compare the lat and lon of the start and ending points to see if valid closed polygon
    LatLon startPoint = getFeaturePoint(feature_id,0);
    LatLon endPoint = getFeaturePoint(feature_id, getNumFeaturePoints(feature_id)-1);
    if(!(startPoint == endPoint)){
        return 0;
    }

    double sum = 0;   
    for(int i = 0; i< getNumFeaturePoints(feature_id)-1; i++){
        //converting to radians and using pythagorean theorem to solve 
        startPoint = getFeaturePoint(feature_id,i);
        endPoint = getFeaturePoint(feature_id, i+1);
        double latAvg = (startPoint.latitude() * kDegreeToRadian + endPoint.latitude() * kDegreeToRadian)/2.0;
        double x1 = startPoint.longitude() * kDegreeToRadian *  cos(latAvg) * kEarthRadiusInMeters;
        double y1 = startPoint.latitude() * kDegreeToRadian * kEarthRadiusInMeters;
        double x2 = endPoint.longitude() * kDegreeToRadian * cos(latAvg) * kEarthRadiusInMeters;
        double y2 = endPoint.latitude() * kDegreeToRadian * kEarthRadiusInMeters;
       
        
        sum += ((x1+x2)/2) * (y2-y1); //formula  
    }
    
    return std::abs(sum);
}
//helper function for bounding box
double checkMin(double point, double currentMin){
    
    if(point<currentMin){currentMin=point;}
    return currentMin;  
}
//helper function for bounding box
double checkMax(double point, double currentMax){
     
    if(point>currentMax){currentMax=point;}
    return currentMax;  
}