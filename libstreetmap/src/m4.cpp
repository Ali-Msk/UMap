/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m4_internal.h"

#define TIME_LIMIT 50

std::unordered_map<IntersectionIdx,std::unordered_map<IntersectionIdx,double>>  allDestinationPaths;
double bestTravelTime; 
std::vector <int> longSubPaths;



std::vector<CourierSubPath> travelingCourier(const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots, const float turn_penalty){
    //start time
    auto startTime = std::chrono::high_resolution_clock::now();
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime- startTime);
    
    
    std::vector<CourierSubPath> ans;
    
    std::vector<IntersectionIdx> allIntersectionIds;
    std:: unordered_set <IntersectionIdx> uniqueInterestingIntersections;
   
    
    std::vector<IntersectionIdx> intersectionOrder;

    
    std::unordered_map <IntersectionIdx, bool> isPickup;
    std::vector<IntersectionIdx> dropOffs;
    std::vector <IntersectionIdx> pickUps;
    std::unordered_map <IntersectionIdx, IntersectionIdx> readyToDropOff; //dropoff to pickup
    
    std::unordered_map <IntersectionIdx, std::vector <IntersectionIdx>> dropOffToPickUp; 
    
    
    
    for (int i =0; i< deliveries.size() ;i++){
        if (dropOffToPickUp.count(deliveries[i].dropOff) == 1){
   //         std::cout<<"addingiggnigignging"  <<std::endl;
            dropOffToPickUp[deliveries[i].dropOff].push_back(deliveries[i].pickUp);
        }else{
            std::vector <IntersectionIdx> temp;
            temp.push_back(deliveries[i].pickUp);
            dropOffToPickUp[deliveries[i].dropOff] = temp;
        }
        
        dropOffs.push_back(deliveries[i].dropOff);
        pickUps.push_back(deliveries[i].pickUp);
        readyToDropOff[deliveries[i].dropOff] = deliveries[i].pickUp;
        isPickup[deliveries[i].pickUp] = true;
        isPickup[deliveries[i].dropOff] = false;
    }
    
   
    for(int i = 0; i< deliveries.size(); i++){
        uniqueInterestingIntersections.insert(deliveries[i].pickUp);
        uniqueInterestingIntersections.insert(deliveries[i].dropOff);
    }

    
    for(int i = 0; i< depots.size(); i++){
        uniqueInterestingIntersections.insert(depots[i]);
    }
    
    //go through the delivery intersections and depots to pre-load path with multi threading
    for(int i : uniqueInterestingIntersections){
        allIntersectionIds.push_back(i);
    }
    
    int size = ceil(allIntersectionIds.size());
    
    #pragma omp parallel for
    for(int i = 0; i < size; i++){
       //auto startTime = std::chrono::high_resolution_clock::now();
  // auto currentTime = std::chrono::high_resolution_clock::now();
  //  auto wallClockStart = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime- startTime);
 //     
        multiDestDijstra(allIntersectionIds[i],deliveries,depots,turn_penalty,uniqueInterestingIntersections.size(), isPickup);
       //currentTime = std::chrono::high_resolution_clock::now();
  // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime- startTime);
  //std:: cout << "total Compute Time  " << i << " " << wallClock.count() - wallClockStart.count()  << std::endl;
    }


    std::vector<std::vector<CourierSubPath>> allPath;
    allPath.resize(depots.size());
    
    ans = findCourierPath(depots[0], deliveries, depots, turn_penalty,1);
    allPath[0]= ans;
    bestTravelTime = calucauteTotalTravelTime(allPath[0]);
    

    #pragma omp parallel for
    for(int i= 0; i<depots.size(); i++){
        double bestCurrentTime = std::numeric_limits<double>::max();
        int og = 1;
        auto currentTime2 = std::chrono::high_resolution_clock::now();
        auto wallClock2 = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime2- startTime);
        do{
            std::vector<CourierSubPath> temp = findCourierPath(depots[i], deliveries, depots, turn_penalty, og);
            og = 0;
            double travelTime = calucauteTotalTravelTime(temp);
            if(travelTime < bestCurrentTime){
                bestCurrentTime = travelTime;
                allPath[i] = temp;
            }
            currentTime2 = std::chrono::high_resolution_clock::now();
            wallClock2 = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime2- startTime);
        } while(wallClock2.count() < 0.5 * TIME_LIMIT && deliveries.size() > 98);
        
    }
    

    
    
    
    for(int i = 1; i<allPath.size(); i++){
        double travelTime = calucauteTotalTravelTime(allPath[i]);
        
        if(travelTime < bestTravelTime){
            ans = allPath[i];
            bestTravelTime = travelTime;
        }     
    
    }
    
    for (int i = 0; i < ans.size(); i ++ ){
        if(allDestinationPaths[ans[i].start_intersection][ans[i].end_intersection] > 60 * 6){
            longSubPaths.push_back(i);
        }

    }
//    int deleted = 0;
//    std::unordered_set<int> alreadySeen;
//    for (int i = 0; i < longSubPaths.size(); i ++ ){
//        if(alreadySeen.count(longSubPaths[i - deleted]) == 1){
//            longSubPaths.erase(longSubPaths.begin() + i - deleted);
//            deleted ++;
//        }else{
//            alreadySeen.insert(longSubPaths[i-deleted]);
//        }
//        
//    }
    
   // std::cout<< longSubPaths.size() <<std::endl;
    int count = 0;
    bool timeOut = false;
    bool doneSmallPerturbations = false;
    bool doneLargePerturbations = false;
    while(!timeOut){
        if(count > longSubPaths.size()){
            doneLargePerturbations = true;
        }
        if(doneLargePerturbations && doneSmallPerturbations){
     //       std::cout<< count <<std::endl;
            timeOut = true;
        }
       
      //void perturbLongPaths(std::vector<CourierSubPath> &path, int index, int lookAheadNum, double time,  
        //double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots); 
        int answerSize = ans.size();
       if(count < longSubPaths.size() && count < answerSize - 6 &&  longSubPaths[count]< answerSize - 6){
         // std::cout<<"a " << longSubPaths[0] << " "  << ans.size() << " "<< count<<std::endl;
          // std::cout << "checking permutation"<<std::endl;
          perturbLongPaths(ans, longSubPaths[count], 4, bestTravelTime, turn_penalty, deliveries, depots);
       }
       
        std::pair<std::vector<CourierSubPath>, double>  newResult;
        if(count < ans.size() - 4  && !doneSmallPerturbations){
            newResult = perturbation( ans, count + 1 , bestTravelTime,  turn_penalty,  deliveries, depots);
        }
        if(newResult.first.size() != 0){
            ans = newResult.first;
            bestTravelTime = newResult.second;
        }else{
            //do other purterbations
          doneSmallPerturbations = true;
        }
       
         currentTime = std::chrono::high_resolution_clock::now();
         wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime- startTime);
        if(wallClock.count() > 0.94 * TIME_LIMIT){
      //      std::cout<< count <<std::endl;
            timeOut = true;
        }
       //timeOut = true;
        count ++;
    }
    
    
    
    
    //intersectionOrder.push_back(depots[depotIndex]);
    return ans;
}

std::vector<CourierSubPath> findCourierPath(IntersectionIdx begin, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots, const float turn_penalty, int og){
    
    std::unordered_set<IntersectionIdx> dropOffs;
    std::unordered_set <IntersectionIdx> pickUps;
    std::unordered_map <IntersectionIdx, IntersectionIdx> readyToDropOff; //dropoff to pickup
    std::unordered_map <IntersectionIdx, std::unordered_set <IntersectionIdx>> dropOffToPickUp; 
    
    for (int i =0; i< deliveries.size() ;i++){
        if (dropOffToPickUp.count(deliveries[i].dropOff) == 1){
            dropOffToPickUp[deliveries[i].dropOff].insert(deliveries[i].pickUp);
        }else{
            std::unordered_set <IntersectionIdx> temp;
            temp.insert(deliveries[i].pickUp);
            dropOffToPickUp[deliveries[i].dropOff] = temp;
        }
        
        
        dropOffs.insert(deliveries[i].dropOff);
        pickUps.insert(deliveries[i].pickUp);
        
        readyToDropOff[deliveries[i].dropOff] = deliveries[i].pickUp;
    }
    
    std::vector<CourierSubPath> ans;
    
    std:: unordered_set <IntersectionIdx> alreadyPickedUp;
    std:: unordered_set <IntersectionIdx> alreadyDroppedOff;
    
    bool first = true;
    int current = begin;
    std::vector<DeliveryInf> toDeliver;
    
     while(dropOffs.size() > 0){
        IntersectionIdx closest = -1;
        double closestTime = std::numeric_limits<double>::max();
        int closestIsPickUp = 2;
        std::vector <int> closestPickUpIndex;

        int secondClosest = -1;
        int secondClosestIsPickUp = 2;
        
        for(int i : pickUps){
            if(allDestinationPaths[current][i]  < closestTime){
                secondClosest = closest;
                secondClosestIsPickUp = closestIsPickUp;
                
                closestTime = allDestinationPaths[current][i];
                closest = i;
                closestIsPickUp = 1;
            }
        }
        
        if(!first){
            for(int i : dropOffs){
                std::vector <int> temp = checkIfPickedUp(i, dropOffToPickUp,alreadyPickedUp);
                 if(temp.size() != 0 && allDestinationPaths[current][i]  < closestTime ){  
                    secondClosest = closest;
                    secondClosestIsPickUp = closestIsPickUp;
                    
                    closestTime = allDestinationPaths[current][i] ;
                    closest = i;
                    closestPickUpIndex = checkIfPickedUp(i, dropOffToPickUp,alreadyPickedUp);
                    closestIsPickUp  = 0;
                 }
            }
        }
        
       if(closest == -1 ){
            break;
       }
        
        first = false;

        int num  = rand() % 10 + 1;
        //std:: cout << num << std::endl;
        if(secondClosest != -1 && og == 0){
            if(num == 6){
                if(secondClosestIsPickUp == 1){
                    closest = secondClosest; 
                    closestIsPickUp = secondClosestIsPickUp;
                }else{
                    closest = secondClosest; 
                    closestPickUpIndex = checkIfPickedUp(closest, dropOffToPickUp,alreadyPickedUp);
                    closestIsPickUp = 0;
                }
                //std:: cout <<"pooop";
            }
        }
        
        
        CourierSubPath currentSubPath;
        currentSubPath.start_intersection = current;
        currentSubPath.end_intersection = closest;
        
        if(closestIsPickUp == 1){
            alreadyPickedUp.insert(closest);
            pickUps.erase(closest);
        }else if(closestIsPickUp == 0){
            alreadyDroppedOff.insert(closest);
            for (int i = 0; i < closestPickUpIndex.size(); i++){
                dropOffToPickUp[closest].erase(closestPickUpIndex[i]);
            }
            
            if(dropOffToPickUp[closest].size() == 0){
                dropOffs.erase(closest);
            }
        }

    //    if(allDestinationPaths[current][closest] > 60 * 27.5){ //if longer than 15 mins add to long paths

     //       if(ans.size() != 0){
      //          longSubPaths.push_back(ans.size());
     //       }
     //   }
        
        currentSubPath.subpath = findPathBetweenIntersections(current, closest, turn_penalty);
        ans.push_back(currentSubPath);
        current = closest;
     }
    
    double minTime = 9999999;
    int depotIndex = 0;
    for(int i = 0; i<depots.size(); i++){
        double travelTime = allDestinationPaths[current][depots[i]];
        if(travelTime < minTime){
            depotIndex = i;
            minTime = travelTime;
        }
      
    }
    
    
    CourierSubPath currentSubPath;
    currentSubPath.start_intersection = current;
    currentSubPath.end_intersection = depots[depotIndex];
    currentSubPath.subpath = findPathBetweenIntersections(current, depots[depotIndex], turn_penalty);
    
    ans.push_back(currentSubPath);
    
    return ans;
    
    
}

std::vector<int>  checkIfPickedUp(int id, std::unordered_map <IntersectionIdx, std::unordered_set <IntersectionIdx>> dropOffToPickUp, std:: unordered_set <IntersectionIdx> alreadyPickedUp){
  //  std::cout<<"11"  <<std::endl;
    std::vector<int> result; 
    if(dropOffToPickUp.count(id) == 0){
        return result;
    }

    for (int i :dropOffToPickUp[id]){
        if(alreadyPickedUp.count(i) == 1){
  //          std::cout<<"dropping"<<dropOffToPickUp[id][i] <<std::endl;
            result.push_back(i);
            //return i;
        }
    }
    return result;
    //return -1;
}


void multiDestDijstra(int intersect_id_start,const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, const float turn_penalty, int numInterestingIntersections, std::unordered_map <IntersectionIdx, bool> isPickup){
     //intialize variables
    std::vector<StreetSegmentIdx> shortestPath;
    
    //declare a vector of intersectionNodes of structs containing interesection info
    std::vector<IntersectionNode> intersectionNodes; 
    intersectionNodes.resize(getNumIntersections());
    TIME_PENALTY = turn_penalty;
    //declare a priority queue with the lowest value set to the front
    std:: priority_queue<std::pair<double, int>,std::vector<std::pair<double, int>>,std::greater<std::pair<double, int>> > queue;
    
    //add the initial intersection to the queue
    queue.push(std:: make_pair(0,intersect_id_start));
    intersectionNodes[intersect_id_start].travelTime = 0;
    
    std:: unordered_set <IntersectionIdx> vistedUniqueIntersections; 

    //auto startTime2 = std::chrono::high_resolution_clock::now();
  //auto currentTime2 = std::chrono::high_resolution_clock::now();
   //auto wallClockStart = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime2- startTime2);
    while(!queue.empty()){     
        //get the currently best path and pop it from the queue
        IntersectionIdx current = queue.top().second;
        queue.pop();    

        //get near neighbors of current and store it inside vector
        std:: vector<int> adjacentIntersections = findAdjacentIntersections(current);
        
        int prev = -1;
        //get previous segment
        if(intersect_id_start != current){
            prev = intersectionNodes[current].previousSegment;            
        }
                
        //exit condition when all interesting intersections has been visted        
        if(isPickup.count(current) != 0){
            vistedUniqueIntersections.insert(current);
        }
      
        for(int i =  0; i<depots.size(); i++){
            if(current == depots[i]){
               vistedUniqueIntersections.insert(current);
            }
        }
        //std::cout << vistedUniqueIntersections.size() << std::endl;
        if(vistedUniqueIntersections.size() == numInterestingIntersections ){
            //std:: cout <<"exiasdfasdfted";
            break;
       }

        //loop through adjacent intersections and add them to the queue inorder
        for(int i = 0; i<adjacentIntersections.size(); i++){
            IntersectionIdx next = adjacentIntersections[i];
                if(intersectionNodes[next].closed == 0){ 
                    //get time taken to get to current intersection
                    double currentTravelTime = findIntersectionTraveltime(current,next,prev) + intersectionNodes[current].travelTime;                                      
                     
                    //if not beginning intersection, check for turns, add turn penalty
                    if(prev != -1 &&  checkTurn(current, next,prev)){ 
                        currentTravelTime += turn_penalty;
                    }
                     
                    double currentPriority = currentTravelTime;
                    
                    //check if the f value is less, if so update
                    if(currentPriority >= intersectionNodes[next].travelTime){ //intersectionNodes[next].visited &&
                        continue;
                    }
                    
                    //push pair of adjacent intersection into queue with calculated priority 
                    queue.push(std:: make_pair(currentPriority,next));

                    //update traveltime and heuristic value in intersectionNodes vector 
                    intersectionNodes[next].travelTime = currentTravelTime;
 
                    //set the path which the edge came from 
                    intersectionNodes[next].previousIntersection = current;
                    intersectionNodes[next].previousSegment = findStreetSegment(current,next,prev);     
               }    
        }  
        //close the intersection after going through all its neighbours
        intersectionNodes[current].closed = 1; 
    }     
       // currentTime2 = std::chrono::high_resolution_clock::now();
   // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime2- startTime2);
 //std:: cout << "while loop Time " << wallClock.count() - wallClockStart.count()  << std::endl;
    
    //compute travel time for all interesting intersections   
    for(int i = 0; i<deliveries.size(); i++){
        //check if any interesting deliveries points have been reached
        int intersect_id_destination = deliveries[i].dropOff;
        shortestPath = backTrack(intersectionNodes,intersect_id_destination);
        double travelTime = computePathTravelTime(shortestPath,turn_penalty);  
        if(travelTime == 0){
            travelTime = std::numeric_limits<double>::max();
        }

        #pragma omp critical
        allDestinationPaths[intersect_id_start][intersect_id_destination] = travelTime;
        
        intersect_id_destination = deliveries[i].pickUp;
        shortestPath = backTrack(intersectionNodes,intersect_id_destination);
        travelTime = computePathTravelTime(shortestPath,turn_penalty); 
        if(travelTime == 0){
            travelTime = std::numeric_limits<double>::max();
        }
        
        #pragma omp critical
        allDestinationPaths[intersect_id_start][intersect_id_destination] = travelTime;
    }
    
    for(int i = 0; i<depots.size(); i++){
        int intersect_id_destination = depots[i];
        shortestPath = backTrack(intersectionNodes,intersect_id_destination);
        double travelTime = computePathTravelTime(shortestPath,turn_penalty); 
        if(travelTime == 0){
            travelTime = std::numeric_limits<double>::max();
        }
        
        #pragma omp critical
        allDestinationPaths[intersect_id_start][intersect_id_destination] = travelTime;
    }
    
    //return empty vector is no path is found
    return;
}

//function that selects the optimal starting depot
IntersectionIdx findStartingLocation(const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots){
    IntersectionIdx startingDepot = -1;
    double shortestTime = std::numeric_limits<int>::max();
    
    //iterates through depots and deliveries and selects depot with shortest travel time to a pickup location
    for(int i = 0; i<depots.size(); i++){
        for(int j = 0; j<deliveries.size(); j++){
            //updates starting depot if travel time is minimumm
            IntersectionIdx depotLocation = depots[i];
            IntersectionIdx deliveryLocation = deliveries[i].pickUp;
            if(allDestinationPaths[depotLocation][deliveryLocation] < shortestTime){
                startingDepot = depotLocation;
                shortestTime = allDestinationPaths[depotLocation][deliveryLocation];
            }
        }
    }
    
    return startingDepot;
}

//function that checks the legality of a path
bool checkLegalPath(std::vector<CourierSubPath> path, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots){

    std:: unordered_set <IntersectionIdx> alreadyPickedUp;
    std:: unordered_set <IntersectionIdx> alreadyDroppedOff;
    
    //iterate through path
    for(int k=0; k<path.size(); k++){
        
        CourierSubPath current = path[k];
        CourierSubPath prev;
        
        if(k!=0){
            prev = path[k-1];
            
            //check if previous segment connects to current one
            if(current.start_intersection!= prev.end_intersection){
                return false;
            }
            //check if end is a depot, if not return false
            if(k==path.size()-1){
                if(std::find(depots.begin(), depots.end(), current.end_intersection) == depots.end()){
                    return false;
                }
            }
        }
        //check if start is a depot, if not return false
        else{
            if(std::find(depots.begin(), depots.end(), current.start_intersection) == depots.end()){
                    return false;
            }
        }
        
        //check if intersection is a pickup or dropoff, check if path violates pickup dropoff order of deliveries
        for(int j=0; j<deliveries.size(); j++){
            if(current.start_intersection==deliveries[j].pickUp){
                alreadyPickedUp.insert(current.start_intersection);
            }
            else if(current.start_intersection==deliveries[j].dropOff){
                //if not already picked up, return false 
                //account for possibility that are multiple pickups with the same dropoff
                bool pickUpCheck= false;
                for(int i=0; i<deliveries.size(); i++){
                    if(deliveries[i].dropOff == deliveries[j].dropOff && alreadyPickedUp.count(deliveries[i].pickUp)==1){
                        pickUpCheck= true;
                    }    
                }
                
                if(!pickUpCheck){
                    return false; //hasnt been picked up
                }
                
                alreadyDroppedOff.insert(current.start_intersection);
            }
        }
    } 
    
    //now iterate through deliveries, and ensure they have all been picked up and dropped of
    for(int i=0; i<deliveries.size();i++){
        //check if all deliveries have been picked up
        if(alreadyPickedUp.count(deliveries[i].pickUp)==0){
            return false;
        }
        //check if all deliveries have been dropped off
        if(alreadyDroppedOff.count(deliveries[i].dropOff)==0){
            return false;
        }

    }
    
    return true; //if hasn't returned by now its a legal path, return true
}

//check if legal function 
/*
bool checkDropoff(std:: unordered_set <DeliveryInf>& alreadyPickedUp, DeliveryInf drop){
    for (int i = 0; i< alreadyPickedUp.size() ; i++){
        if (drop.dropOff == alreadyPickedUp[i].dropOff){
            return true;
        }       
    }
    return false;
    
}*/

std::pair<std::vector<CourierSubPath>, double> perturbation(std::vector<CourierSubPath> path, int i, double time, double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots){
    bool changed = false;
    double newTime = time;
    std::vector<CourierSubPath> empty;
    for (int j = 1 ; j< path.size()-3; j++){
     //   std::cout <<"3"<<std::endl;
        std::pair <std::vector<CourierSubPath>, double> newPath= trySwap(path, j, newTime, turn_penalty, deliveries, depots);
      //  std::cout <<"4"<<std::endl;
        if( newPath.first.size() != 0){
            path = newPath.first;
            newTime = newPath.second;
            changed = true;
        }
       // newPath= trySwap2(path, i, newTime, turn_penalty, deliveries, depots);
     //   if( newPath.first.size() != 0)
      //      path = newPath.first;
     //       newTime = newPath.second;
      //      changed = true;
        }
        
  //  }
    if(changed){
        return std::make_pair(path, newTime);
    }
    return std::make_pair(empty, 0);
}
std::pair<std::vector<CourierSubPath>, double> trySwap2(std::vector<CourierSubPath> path,int i ,double time,  double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots){
    CourierSubPath first;
    CourierSubPath  second;
    CourierSubPath third; 
    first.start_intersection = path[i].start_intersection;
    first.end_intersection = path[i+2].end_intersection;
    first.subpath = findPathBetweenIntersections(first.start_intersection, first.end_intersection, turn_penalty);
    second.start_intersection = path[i+2].end_intersection;
    second.end_intersection = path[i+1].end_intersection;
    second.subpath = findPathBetweenIntersections(second.start_intersection, second.end_intersection, turn_penalty);
    third.start_intersection = path[i+1].end_intersection;
    third.end_intersection = path[i].end_intersection;
    third.subpath = findPathBetweenIntersections(third.start_intersection, third.end_intersection, turn_penalty);
    
    path[i] = first;
    path[i+1] = second;
    path[i+2] = third;
    
    std::vector<CourierSubPath> empty;
    double newTime = 0;
 //   std::cout << "checking for legal"<<std::endl;
    if(checkLegalPath(path, deliveries, depots)){
        newTime = calucauteTotalTravelTime(path);
    //    std::cout << "legal"<<std::endl;
        if(newTime < time){
    ////        std::cout << "improvement"<<std::endl;
            return std::make_pair(path, newTime);

        }
    }
    return std::make_pair(empty, 0);
    
}
std::pair<std::vector<CourierSubPath>, double> trySwap(std::vector<CourierSubPath> path,int i ,double time,  double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots){
    CourierSubPath first;
    CourierSubPath  second;
    CourierSubPath third; 
   // std::cout <<third.start_intersection<<"  "  <<  third.end_intersection<<std::endl;
    first.start_intersection = path[i].start_intersection;
    first.end_intersection = path[i+2].start_intersection;
    first.subpath = findPathBetweenIntersections(first.start_intersection, first.end_intersection, turn_penalty);
    second.start_intersection = path[i+2].start_intersection;
    second.end_intersection = path[i+1].start_intersection;
    second.subpath = findPathBetweenIntersections(second.start_intersection, second.end_intersection, turn_penalty);
    third.start_intersection = path[i+1].start_intersection;
    third.end_intersection = path[i+2].end_intersection;
   // std::cout <<third.start_intersection<<"  "  <<  third.end_intersection<<std::endl;
    third.subpath = findPathBetweenIntersections(third.start_intersection, third.end_intersection, turn_penalty);
    path[i] = first;
    path[i+1] = second;
    path[i+2] = third;
    
    std::vector<CourierSubPath> empty;
    double newTime = 0;
 //   std::cout << "checking for legal"<<std::endl;
    if(checkLegalPath(path, deliveries, depots)){
        newTime = calucauteTotalTravelTime(path);
    //    std::cout << "legal"<<std::endl;
        if(newTime < time){
    ////        std::cout << "improvement"<<std::endl;
            return std::make_pair(path, newTime);

        }
    }
    return std::make_pair(empty, 0);
    
}

double calucauteTotalTravelTime(std::vector<CourierSubPath> &path){
    double travelTime = 0;
    
    if(path.size() == 0){
        return std::numeric_limits<double>::max(); 
    }
    
    for(int i = 0; i<path.size(); i++){
        CourierSubPath curr = path[i];
        travelTime += allDestinationPaths[curr.start_intersection][curr.end_intersection];
    }
    
    if(travelTime == 0){
        travelTime = std::numeric_limits<double>::max();
    }
    
    return travelTime;
}





void perturbLongPaths(std::vector<CourierSubPath> &path, int index, int lookAheadNum, double time,  
        double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots){
    
    for (int i = 0 ; i < lookAheadNum; i++ ){
        std::vector<int> current;
        std::unordered_set<int> used;
        used.insert(i);
        current.push_back(i);
    //    std::cout << "1"<<std::endl;
        perturbLongPathsHelper(path, index, lookAheadNum, time, turn_penalty, deliveries, depots, current, used);
   //     std::cout << "2"<<std::endl;
    }
    std::vector<int> current;
    std::unordered_set<int> used;
    
}
void perturbLongPathsHelper(std::vector<CourierSubPath> &path, int index, int lookAheadNum, double time,  
        double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots,
        std::vector<int> current, std::unordered_set<int> used){
    if(current.size() == lookAheadNum){
//        double totalTime = 0 ;
       // std::cout << "checking permutation"<<std::endl;
      //  for (int i = 0 ; i< current.size()-1; i ++ ){
       //    totalTime += allDestinationPaths[path[index+current[i]].start_intersection][ path[index+current[i+1]].start_intersection];

            
            std::vector <CourierSubPath> tempPath;

            //[1,2,3,0]
           // std::cout << "a"<<std::endl;
            for ( int i= 0 ; i < current.size(); i ++){
                CourierSubPath subpath;
                if(i == 0){
                    subpath.start_intersection = path[index].start_intersection;
                    subpath.end_intersection = path[index+current[i]+1].start_intersection;  
                }else{
                    subpath.start_intersection = path[index+current[i-1]+1].start_intersection;
                    subpath.end_intersection = path[index+current[i]+1].start_intersection;  
                }
                //subpath.end_intersection = path[index+current[i+1]].start_intersection;  
                subpath.subpath = findPathBetweenIntersections(subpath.start_intersection, subpath.end_intersection, turn_penalty);
                //path[index + i ] = subpath;
                tempPath.push_back(subpath);
            }
          //  std::cout << "b"<<std::endl;
            CourierSubPath subpath;//std::cout << "e"<<std::endl;
            subpath.start_intersection = path[index+current[current.size()-1]+1].start_intersection;//std::cout << "f"<<std::endl;
            subpath.end_intersection = path[index+current.size()+1].start_intersection;  //std::cout << "g"<<std::endl;
            //std::cout <<subpath.start_intersection << "h"<< subpath.end_intersection<< " " << path.size() << " " << index+current.size()+1<< std::endl;
            subpath.subpath = findPathBetweenIntersections(subpath.start_intersection, subpath.end_intersection, turn_penalty);
           // std::cout << "h"<<std::endl;//////////////////
            //path[index + current.size()-1] =  subpath;
            tempPath.push_back(subpath);
          //  std::cout << "i"<<std::endl;
            if(checkLegalSubPath(tempPath, deliveries)){
               // std::cout << "legal"<<std::endl;
                double currentTime = calucauteTotalTravelTime(tempPath);
                std::vector <CourierSubPath> currentTempPath; 
                
                
                for (int i  = index ; i < index + tempPath.size(); i++){
                    currentTempPath.push_back(path[i]);
                }
                
                time = calucauteTotalTravelTime(currentTempPath);
             //   std::cout << "checking: " << time<< " " <<  currentTime<<std::endl;
                if(currentTime < time){
                  //  std::cout << "improvemet: " << time<< " " <<  currentTime<<std::endl;
                    for(int i =0 ; i < tempPath.size();i++){
                        path[index + i] = tempPath[i];
                    }
                }
            }
            
       // }
        return;
    }
   // std::cout << "3"<<std::endl;
    for (int i = 0; i< lookAheadNum; i++){
        int num  = rand() % 10 + 1;
        if(used.count(i) != 1){
            if(num <= 5){
                std::vector < int> temp = current; 
                std::unordered_set < int> tempUsed = used;

                temp.push_back(i);
                tempUsed.insert(i);
             //   std::cout << "4"<<std::endl;
                perturbLongPathsHelper(path, index, lookAheadNum, time, turn_penalty, deliveries, depots, temp, tempUsed);
            //   std::cout << "5"<<std::endl;
            }
        }
    }
    
}

//function that checks the legality of a sub path
bool checkLegalSubPath(std::vector<CourierSubPath> path, const std::vector<DeliveryInf>& deliveries){
  //  std::cout << "c"<<std::endl;
    std:: unordered_set <IntersectionIdx> alreadyPickedUp;
    std:: vector <IntersectionIdx> subPathInt;
    
    for(int k=0; k<path.size();k++){
        subPathInt.push_back(path[k].start_intersection);
    }
    //std::cout << "d"<<std::endl;
    for(int k=0; k<path.size();k++){
        CourierSubPath current = path[k];
        
        for(int j=0; j<deliveries.size(); j++){
            
            //check if pickup or dropoff
            if(current.start_intersection==deliveries[j].pickUp){
                alreadyPickedUp.insert(current.start_intersection);
            }
            
            else if(current.start_intersection==deliveries[j].dropOff){
                
                //check if associated pickup is in path before seeing if its been picked up
                if(std::find(subPathInt.begin(), subPathInt.end(), deliveries[j].pickUp) != subPathInt.end()){
                    
                    //if not already picked up, return false 
                    //account for possibility that are multiple pickups with the same dropoff
                    bool pickUpCheck= false;
                    for(int i=0; i<deliveries.size(); i++){
                        if(deliveries[i].dropOff == current.start_intersection && alreadyPickedUp.count(deliveries[i].pickUp)==1){
                            pickUpCheck= true;
                        }    
                    }
                    
                    if(!pickUpCheck){
                        return false; //hasnt been picked up
                    }
                }
            }
        }
    }
    return true;
}
