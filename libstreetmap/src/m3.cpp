/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

//includes 
#include "m3_internal.h"
#include <unordered_set>

//defining global variables  
bool showNavigation = false; //bool that captures state of navigation mode
std::string directions; //directions string
double TIME_PENALTY=15; //time penalty constant
const int SMALLESTZOOMARROWS=50000; //zoom level constants for displaying arrows
const int SMALLZOOMARROWS=500000;
const int MEDZOOMARROWS=5000000;
const int LARGEZOOMARROWS=50000000;
std::vector<StreetSegmentIdx> navigationPath; //navigation path used to draw
std::vector <IntersectionIdx> sharedIntersectionsNav; //intersections vector for navigation mode when drawing path
IntersectionIdx start; //start and end for one segment paths
IntersectionIdx end;

//return time required to travel along the path specified in seconds
double computePathTravelTime(const std::vector<StreetSegmentIdx>& path,const double turn_penalty){
    
    double travelTime = 0.0;
    int currentStreetId = 0;
    int previousStreetId = 0;      
    //loop through the path to determine speed
    for(int i=0; i<path.size(); i++){
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(path[i]);
        //adding length of street accordingly
        double length = findStreetSegmentLength(path[i]);
        travelTime += (length / (double)segmentInfo.speedLimit);
        currentStreetId = segmentInfo.streetID;      
        //check if turn_penalty applies 
        if(i != 0 && currentStreetId != previousStreetId){
            travelTime += turn_penalty; 
        }      
        previousStreetId = currentStreetId;     
    }
    return travelTime;
}

//find the optimal path between two given intersections
std::vector<StreetSegmentIdx> findPathBetweenIntersections(const IntersectionIdx intersect_id_start,const IntersectionIdx intersect_id_destination,const double turn_penalty){

    //intialize variables
    std::vector<StreetSegmentIdx> shortestPath;
    //declare a vector of intersectionNodes of structs containing interesection info
    std::vector<IntersectionNode> intersectionNodes; 
    intersectionNodes.resize(getNumIntersections()+1);
    TIME_PENALTY = turn_penalty;
    //declare a priority queue with the lowest value set to the front
    std:: priority_queue<std::pair<double, int>,std::vector<std::pair<double, int>>,std::greater<std::pair<double, int>> > queue;
    
    //add the initial intersection to the queue
    queue.push(std:: make_pair(0,intersect_id_start));
    intersectionNodes[intersect_id_start].heuristic = 0;
    intersectionNodes[intersect_id_start].travelTime = 0;

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
        
        //if current intersection equals destination then backtrack and return path
        if(current ==  intersect_id_destination){
            shortestPath = backTrack(intersectionNodes,intersect_id_destination);
            return shortestPath;
        } 
        
        //loop through adjacent intersections and add them to the queue inorder
        for(int i = 0; i<adjacentIntersections.size(); i++){
            IntersectionIdx next = adjacentIntersections[i];
                if(intersectionNodes[next].closed == 0){ 
                    //get time taken to get to current intersection
                    double currentTravelTime = findIntersectionTraveltime(current,next,prev) + intersectionNodes[current].travelTime;                                      
                    
                    //if path so far took longer than time to get to destination (if any), go next
                    if(currentTravelTime >= intersectionNodes[intersect_id_destination].travelTime){
                        continue;
                    }    
                    //if not beginning intersection, check for turns, add turn penalty
                    if(prev != -1 &&  checkTurn(current, next,prev)){ 
                        currentTravelTime += turn_penalty;
                    }
                     
                    //get heuristic to calculate priority
                    double heuristicValue = findHeuristic(next,intersect_id_destination);
                    double currentPriority = currentTravelTime + heuristicValue;
                    
                    //check if the f value is less, if so update
                    if(currentPriority >= intersectionNodes[next].travelTime + intersectionNodes[next].heuristic){ //intersectionNodes[next].visited &&
                        continue;
                    }
                    
                    //push pair of adjacent intersection into queue with calculated priority 
                    queue.push(std:: make_pair(currentPriority,next));

                    //update traveltime and heuristic value in intersectionNodes vector 
                    intersectionNodes[next].travelTime = currentTravelTime;
                    intersectionNodes[next].heuristic = heuristicValue;

                    //set the path which the edge came from 
                    intersectionNodes[next].previousIntersection = current;
                    intersectionNodes[next].previousSegment = findStreetSegment(current,next,prev); 
                    
                }    
        }  
        //close the intersection after going through all its neighbours
        intersectionNodes[current].closed = 1; 
    }               
    
    //return empty vector is no path is found
    return shortestPath;
    
}

//check if there is a from current to next segment and prev streetSegment
bool checkTurn(IntersectionIdx current, IntersectionIdx next, StreetSegmentIdx prev){
    //return true if turn;

    //find the segment between current and next
    int nextSegment = findStreetSegment(current, next,prev);
    StreetSegmentInfo segmentInfoPrev = getStreetSegmentInfo(prev); 
    StreetSegmentInfo segmentInfoNext = getStreetSegmentInfo(nextSegment); 
    
    //get IDs
    StreetIdx first = segmentInfoPrev.streetID;
    StreetIdx second = segmentInfoNext.streetID;

    //compare id of street segments to see if it turns
    if(first != second ){
        return true;
    }
    return false;
}

//find best street segment between two intersections
StreetSegmentIdx findStreetSegment(int firstIntersection, int secondIntersection, StreetSegmentIdx previousSegment){  
    std:: vector<StreetSegmentIdx> adjacentStreetSegments = findStreetSegmentsOfIntersection(firstIntersection);
    
    //contains all legal path of the two intersections
    std:: vector<StreetSegmentIdx> legalPath;
    double bestTravelTime = std::numeric_limits<int>::max();
    StreetSegmentIdx bestSegment = -1;
    
    for(int i = 0; i<adjacentStreetSegments.size(); i++){ 
         
        StreetSegmentIdx currentSegment = adjacentStreetSegments[i]; 
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(currentSegment); 
         
        if(!segmentInfo.oneWay){ 
            //if street is one way then add to legal path if from and to matches 
            if(segmentInfo.from == firstIntersection && segmentInfo.to == secondIntersection){ 
                legalPath.push_back(currentSegment); 
            }else if(segmentInfo.to == firstIntersection && segmentInfo.from == secondIntersection){ 
                legalPath.push_back(currentSegment); 
            } 
        }else{  
            //if one way then add if from matches start and to matches end 
            if(segmentInfo.from == firstIntersection && segmentInfo.to == secondIntersection){ 
                 legalPath.push_back(currentSegment); 
            }else{  
                continue; 
            } 
        } 
    } 

    //go through the legal path to return the best segment
    for(int i = 0; i < legalPath.size(); i++){
        double travelTime = findStreetSegmentTravelTime(legalPath[i]);
        if(previousSegment != -1 ){
            //check if turn penalty applies to the segment
            StreetSegmentInfo segmentInfoPrev = getStreetSegmentInfo(previousSegment); 
            StreetSegmentInfo segmentInfoNext = getStreetSegmentInfo(legalPath[i]);   
            StreetIdx first = segmentInfoPrev.streetID;
            StreetIdx second = segmentInfoNext.streetID;
            if(first != second){
                travelTime += TIME_PENALTY;
            }
        }
        if(travelTime < bestTravelTime){
            bestTravelTime = travelTime;
            bestSegment = legalPath[i];
        }
    }

    return bestSegment;
}

//calculate street segment travel time
double findIntersectionTraveltime(int startIntersection, int endIntersection, int prev){ 
    //get segment 
   StreetSegmentIdx streetSegment = findStreetSegment(startIntersection, endIntersection,prev); 
   //return travel time of the segment
   return findStreetSegmentTravelTime(streetSegment); 
} 

//calculate the heuristic of the intersection
double findHeuristic(int currentIntersection, int endIntersection){ 
    //get euclidean distance between current and end intersecitons 
    double remainingDistance = findDistanceBetweenTwoPoints(std::make_pair(getIntersectionPosition(currentIntersection), getIntersectionPosition(endIntersection)));

    //use 29m/s as average speed
    double speed = 29; 
    
    //return time by dividing 
    return (remainingDistance/speed); 
} 

//back track to recreate the path
std::vector<StreetSegmentIdx> backTrack(std::vector<IntersectionNode> &intersectionNodes, IntersectionIdx destination){ 
    
    std::vector<StreetSegmentIdx> shortestPath;    
    IntersectionNode current = intersectionNodes[destination];
    
    //start from destination node and store which street segment it came from 
    while(current.previousIntersection != -1){ 
        StreetSegmentIdx previous = current.previousSegment;   
        shortestPath.push_back(previous); 
        current  = intersectionNodes[current.previousIntersection]; 
    }
    
    //reverse the vector so vector starts at the starting intersection
    std::reverse(shortestPath.begin(), shortestPath.end());
    
    return shortestPath;   
}
// function that displays path and directions string
void displayDirections(ezgl::renderer *g, std::vector<StreetSegmentIdx> path){
    
    //get current zoom level
    ezgl::rectangle visibleWorld= g->get_visible_world();
    directions = "";
    int intersectionRad=5;
    
    //if path size is 0, then no directions found
    if(path.size()==0){
        directions.append("no directions found, please try again :)");
        return;
    }
    
    //iterate through path
    for(int k=0;k<path.size();k++){ 
         
        // draw path in pink
        g->set_color(ezgl::LIGHT_PINK); 
        g->set_line_width(5);        
        displayStreets(g, path[k]);  
        
        if(k!=0){ 
            
            //draw size and frequency of arrows based on zoom level
            if(visibleWorld.area()<SMALLESTZOOMARROWS){ 
                drawDirectionArrows(g, path[k], path[k-1], 5);
                intersectionRad=5;
            }
            else if(visibleWorld.area()<SMALLZOOMARROWS){
                if(k%2==0){
                    drawDirectionArrows(g, path[k], path[k-1], 10);               
                }  
                intersectionRad=10;
            }
            else if(visibleWorld.area()<MEDZOOMARROWS){
                if(k%5==0){
                    drawDirectionArrows(g, path[k], path[k-1], 40);   
                }
                intersectionRad=15;
            }
            else if(visibleWorld.area()<LARGEZOOMARROWS){
                if(k%10==0){
                    drawDirectionArrows(g, path[k], path[k-1], 80);
                }
                 intersectionRad=20;
            }
            //draw street names
            displayStreetNames(g, getStreetSegmentInfo(path[k-1]), path[k-1]);
        }
        //if k=1, draw start intersection on prev segment, else if end segment draw end intersection and name segment on current segment
        //if only one segment, no need for start and end
        if(path.size()!=1){
            if(k==path.size()-1){ 
                displayStreetNames(g, getStreetSegmentInfo(path[k]), path[k]);
                drawSearchEndPoint(g, path[k-1], path[k], intersectionRad);
            }
            if(k==1){ drawSearchStartPoint(g, path[k-1], path[k], intersectionRad);}
        }
        else{ //draw start and end of one segment path
            g->set_color(ezgl::GREEN);
            g->fill_arc({convertLongitudetoX(getIntersectionPosition(start).longitude()),
            convertLatitudetoY(getIntersectionPosition(start).latitude())}, intersectionRad, 0, 360);
            g->set_color(ezgl::RED);
            g->fill_arc({convertLongitudetoX(getIntersectionPosition(end).longitude()),
            convertLatitudetoY(getIntersectionPosition(end).latitude())}, intersectionRad, 0, 360);
        }
    }
    //get directions now    
    getDirections(g, path);   
}
//function that draws all direction arrows based on zoom level
void drawDirectionArrows(ezgl::renderer *g, StreetSegmentIdx current, StreetSegmentIdx prev, int arrowLen){
   
    StreetSegmentInfo prevSeg= getStreetSegmentInfo(prev);
    StreetSegmentInfo currentSeg= getStreetSegmentInfo(current);
    int numCurve = prevSeg.numCurvePoints;
    
    //find shared intersection, then pass in start and end intersection and start and end curve points based on info
    if((currentSeg.to==prevSeg.to)||(currentSeg.from==prevSeg.to)){
        //starts at previous segment .from, ends at .to, other specs to draw with curve points
        drawArrow(g, prev, prevSeg.from, prevSeg.to, numCurve/2-1, numCurve/2, 0, numCurve, arrowLen);
    }
    else if((currentSeg.to==prevSeg.from)||(currentSeg.from==prevSeg.from)){
        //starts at previous segment .to, ends at .from, other specs to draw with curve points
        drawArrow(g, prev, prevSeg.to, prevSeg.from, numCurve/2, numCurve/2-1, numCurve-1, numCurve, arrowLen);       
    }
}
//function that draws individual direction arrow
void drawArrow(ezgl::renderer *g, StreetSegmentIdx prev, IntersectionIdx segStart, IntersectionIdx segEnd, 
        int curveStart, int curveEnd, int firstCurve, int numCurve, int arrowLen){
    
    double endY=0, endX=0, startY, startX;
    g->set_color(ezgl::MAGENTA);
    
    //find start point, draws arrows at end of previous street segment
    startY= convertLatitudetoY(getIntersectionPosition(segStart).latitude()); 
    startX= convertLongitudetoX(getIntersectionPosition(segStart).longitude());  
    
       //if curve points, use those instead as start and end points for more accuracy in arrowhead
        if(numCurve>1){
            endY=convertLatitudetoY(getStreetSegmentCurvePoint(prev, curveEnd).latitude());
            endX= convertLongitudetoX(getStreetSegmentCurvePoint(prev, curveEnd).longitude());
            startY= convertLatitudetoY(getStreetSegmentCurvePoint(prev, curveStart).latitude());
            startX= convertLongitudetoX(getStreetSegmentCurvePoint(prev, curveStart).longitude()); 
        }
        //if only one curve point, end intersection will be end point
        else if(numCurve>0){
            endY=convertLatitudetoY(getStreetSegmentCurvePoint(prev, firstCurve).latitude());
            endX= convertLongitudetoX(getStreetSegmentCurvePoint(prev, firstCurve).longitude());
        }
    //if no curve points, uses start and intersections and start and end
        else{
            endY=startY+convertLatitudetoY(getIntersectionPosition(segEnd).latitude() 
                    -getIntersectionPosition(segStart).latitude())/2; 
            endX=startX+convertLongitudetoX(getIntersectionPosition(segEnd).longitude() 
                    -getIntersectionPosition(segStart).longitude())/2; 
        }
        
        // set arrow draw parameters, used inspiration from codeguru
        double degrees=50;
        g->set_line_width(3);
        
        // calculate start and points of each arrow segment
        double angle = std::atan2(endY - startY, endX - startX) + M_PI; 
        double x1 = endX + arrowLen * std::cos(angle - degrees); 
        double y1 = endY + arrowLen * std::sin(angle - degrees); 
        double x2 = endX + arrowLen * std::cos(angle + degrees); 
        double y2 = endY + arrowLen * std::sin(angle + degrees); 
        
        //draw arrow
        g->draw_line({endX,endY}, {x1, y1});
        g->draw_line({endX,endY}, {x2, y2});
    
}
// function that gets directions and appends them to a string
void getDirections(ezgl:: renderer *g, std::vector<StreetSegmentIdx> path){
    //variable declarations
    std::vector <StreetIdx> streets; // keeps track of what streets have been drawn already
    std::string temp; //to insert into directions string
    int segmentPathLength=0; //to display segment path time
    g->set_coordinate_system(ezgl::WORLD);  
     
    //iterate through path 
    for(int k=0;k<path.size();k++){ 
        
        //get current street ID and store in vector 
        StreetIdx currentID = getStreetSegmentInfo(path[k]).streetID; 
        streets.push_back(currentID); 
        if(k==0){appendStart(getStreetName(currentID));} 
        
        // if street ID is not the same as the last one inserted into the vector, its a new street 
        if((k!=0)&&streets.size()>1){ 
            if(streets[streets.size()-1]!=streets[streets.size()-2]){ 

                //get last street segment of prev street, and current street segment 
                StreetSegmentInfo prev= getStreetSegmentInfo(path[k-1]); 
                StreetSegmentInfo current= getStreetSegmentInfo(path[k]); 
                
                // append straight directions and distance traveled on last street, reset length 
                appendStraightDistance(prev, segmentPathLength); 
                segmentPathLength=0;               
                
                //get cross product and append turning direction
                double crossProductAngle = getCrossProductAngle(prev, current, path[k-1], path[k]);
                appendTurnDistance(getStreetName(currentID), crossProductAngle); 
            }
        } 
        //update path length
        segmentPathLength+=findStreetSegmentLength(path[k]); 
        //if last segment, add straight distance
        if(k==path.size()-1){ appendStraightDistance(getStreetSegmentInfo(path[k]), segmentPathLength);}
    } 
    //append arrival
    temp = "You have arrived \n";
    directions.append(temp);
    
    //append travel time
}
// function that adds the starting direction to direction string
void appendStart(std::string startName){
    
    // add street of start 
    std::string temp; 
    temp = "Start on "; 
    directions.append(temp); 
    directions.append(startName); 
    temp="\n"; 
    directions.append(temp); 
      
}
// function that adds the direction continue straight to direction string 
void appendStraightDistance(StreetSegmentInfo prev, int segmentPathLength){ 
    
    //add direction and distance travelled 
    std::string temp;
    int kmRounded;
    int mRemaining;
    temp = "Continue straight on ";
    directions.append(temp);
    temp = " for ";
    directions.append(temp); 
    directions.append(getStreetName(prev.streetID));
    directions.append(temp); 
    
    //check if distance over 1000m, display as km if so
    if(segmentPathLength < 1000){
        directions.append(std::to_string(segmentPathLength));
        temp = " m\n";
    }
    else{
        //math to displlay accurate distance
        kmRounded = segmentPathLength/1000;
        mRemaining = segmentPathLength%1000;
        
        directions.append(std::to_string(kmRounded));
        temp = " km and ";
        directions.append(temp);
        directions.append(std::to_string(mRemaining));
        temp = " m\n";
    }
    directions.append(temp);
}

//function that adds turn direction to directions string
void appendTurnDistance(std::string streetName, double crossProductAngle){
    
    std::string temp;
    //negative angle is right, positive is right
    double angleDegrees = crossProductAngle/kDegreeToRadian;
   
    //check if angle is very small, if so its straight
    if((angleDegrees<8)&&(angleDegrees>-8)){
        return;
    }
    //if angle slight larger, its a bear left/right based on sign of angle
    else if((angleDegrees<15)&&(angleDegrees>8)){ 
       temp = "Bear left onto "; 
    } 
    else if((angleDegrees>-15)&&(angleDegrees<-8)){ 
       temp = "Bear right onto "; 
    }
    //angle large enough its an outright turn 
    else if(crossProductAngle>0){ 
        temp = "Turn left onto "; 
    } 
    else if(crossProductAngle<0){ 
        temp = "Turn right onto "; 
    }

    //append to directions
    directions.append(temp);
    directions.append(streetName);
    temp = "\n";
    directions.append(temp);
}

// function that gets cross product angle by using proper vector
double getCrossProductAngle(StreetSegmentInfo prev, StreetSegmentInfo current, StreetSegmentIdx prevIdx, StreetSegmentIdx currentIdx){
    
    double crossProductAngle =0;
    
    //find the shared intersection, and find proper vector based this, the determined start of previous segment, and end of current segment
    //since curve points are accounted for, last curve point position of previous segment and first curve point of current segment are also passed in
    if(prev.to==current.to){
        //shared is prev.to, start is prev.from, end is current.from, start and end curve points are easily determined
        crossProductAngle = getCrossProductAngleWithCorrectPoints(prev.to, prev.from, current.from, current.
                       numCurvePoints-1, prev.numCurvePoints-1, prev, current, prevIdx, currentIdx);
    }
    else if(prev.to==current.from){
        //shared is prev.to, start is prev.from, end is current.to, start and end curve points are easily determined
        crossProductAngle = getCrossProductAngleWithCorrectPoints(prev.to, prev.from, current.to, 0, 
                       prev.numCurvePoints-1, prev, current, prevIdx, currentIdx);
    }
    else if(prev.from==current.from){
        //shared is prev.from, start is prev.to, end is current.from, start and end curve points are easily determined
        crossProductAngle = getCrossProductAngleWithCorrectPoints(prev.from, prev.to, current.to,
                       0, 0, prev, current, prevIdx, currentIdx);
    }
    else{
        //shared is prev.from, start is prev.to, end is current.to, start and end curve points are easily determined
        crossProductAngle = getCrossProductAngleWithCorrectPoints(prev.from, prev.to, current.from, current.
                       numCurvePoints-1, 0, prev, current, prevIdx, currentIdx);
    }

    return crossProductAngle;
}

//function that determines the correct vector to pass in for cross product
double getCrossProductAngleWithCorrectPoints(IntersectionIdx sharedInt, IntersectionIdx startPrev, IntersectionIdx endCurr, 
        int firstCurveCurrent, int lastCurvePrev, StreetSegmentInfo prev, StreetSegmentInfo current,
        StreetSegmentIdx prevIdx, StreetSegmentIdx currentIdx){
        
    double crossProductAngle;
     
    //checks if curve points exist, if so must be taken into account for accuracy, shared point is always shared intersection
    if((prev.numCurvePoints!=0) && (current.numCurvePoints!=0)){
        //both segments have curve points, start is last curve point of prev, end is first curve point of current
        crossProductAngle = calculateCrossProductAngle(intersections[sharedInt].x, intersections[sharedInt].y,
                       convertLongitudetoX(getStreetSegmentCurvePoint(prevIdx, lastCurvePrev).longitude()),
                       convertLatitudetoY(getStreetSegmentCurvePoint(prevIdx, lastCurvePrev).latitude()),
                       convertLongitudetoX(getStreetSegmentCurvePoint(currentIdx, firstCurveCurrent).longitude()),
                       convertLatitudetoY(getStreetSegmentCurvePoint(currentIdx, firstCurveCurrent).latitude()));
                        
    }
    else if(prev.numCurvePoints!=0){
        //only prev has curve points, start is last curve point of prev, end is end intersection of current
        crossProductAngle = calculateCrossProductAngle(intersections[sharedInt].x, intersections[sharedInt].y, 
                       convertLongitudetoX(getStreetSegmentCurvePoint(prevIdx, lastCurvePrev).longitude()),
                       convertLatitudetoY(getStreetSegmentCurvePoint(prevIdx, lastCurvePrev).latitude()),
                       intersections[endCurr].x, intersections[endCurr].y);
    }
    else if(current.numCurvePoints!=0){
        //only current has curve points, end is first curve point of current, start is start intersection of prev
        crossProductAngle = calculateCrossProductAngle(intersections[sharedInt].x, intersections[sharedInt].y,
                       intersections[startPrev].x, intersections[startPrev].y,
                       convertLongitudetoX(getStreetSegmentCurvePoint(currentIdx, firstCurveCurrent).longitude()),
                       convertLatitudetoY(getStreetSegmentCurvePoint(currentIdx, firstCurveCurrent).latitude()));
    }
    else{
        //neither have curve points, startand end are start and end intersections of the prev and current segments
        crossProductAngle = calculateCrossProductAngle(intersections[sharedInt].x, intersections[sharedInt].y,
                       intersections[startPrev].x, intersections[startPrev].y, intersections[endCurr].x, intersections[endCurr].y);
    }
        
    return crossProductAngle;
}

//function that calculates cross product angle
double calculateCrossProductAngle(double Bx, double By, double Ax, double Ay, double Cx, double Cy){
    
    //set up vectors, take cross product
    double prevVector[2] = {By-Ay, Bx-Ax};
    double newVector[2] = {Cy-By, Cx-Bx};
    
    double crossProduct = prevVector[1]*newVector[0] - prevVector[0]*newVector[1];
    double magPrev = std::sqrt(prevVector[0]*prevVector[0]+prevVector[1]*prevVector[1]);
    double magNew = std::sqrt(newVector[0]*newVector[0]+newVector[1]*newVector[1]);
    
    //get angle using magnitudes and asin
    return std::asin((crossProduct)/(magPrev*magNew));
}

// callback function for directions dialog window response signal
void on_dialog_response(GtkWidget *widget, ezgl::application *application){
    
    //destroy window
    gtk_widget_destroy(GTK_WIDGET(widget)); 
    return;
}

// call back function for help window response
void on_help_response(GtkWidget *widget, ezgl::application *application){
    
    //hide help box until user wants to see it again
    gtk_widget_hide(GTK_WIDGET(application->get_object("helpDialog")));
    return;
}

//callback for navigation mode button
void navigate_map(GtkWidget *widget, ezgl::application *application){
    
    //toggles boolean of navigation mode
    showNavigation=!showNavigation;
    
    if(showNavigation){
        //shows search bars and hides find feature
        gtk_widget_show(GTK_WIDGET(application->get_object("navEntry1"))); 
        gtk_widget_show(GTK_WIDGET(application->get_object("navEntry2")));
        gtk_widget_hide(GTK_WIDGET(application->get_object("textInput")));
    }
    else{
        //hides search bars and shows find feature
        gtk_widget_hide(GTK_WIDGET(application->get_object("navEntry1")));
        gtk_widget_hide(GTK_WIDGET(application->get_object("navEntry2")));
        gtk_widget_show(GTK_WIDGET(application->get_object("textInput")));
    }
    showDirections=false;
    application->refresh_drawing();
}

// function that creates dialog window with directions
void popupDirections(ezgl::application *application){
    
    GObject *window;            // the parent window over which to add the dialog
    GtkWidget *content_area;    // the content area of the dialog (i.e. where to put stuffin the dialog)
    GtkWidget *label;           // the label we will create to display a message in thecontent area
    GtkWidget *dialog;          // the dialog box we will create
    GtkWidget *scrolled_window = gtk_scrolled_window_new (NULL, NULL); // the scrolled window
    
    // get a pointer to the main application window
     window = application->get_object(application->get_main_window_id().c_str());
    
    // Create the dialog window. 
    dialog = gtk_dialog_new_with_buttons("Directions",(GtkWindow*) window,GTK_DIALOG_DESTROY_WITH_PARENT,
            ("OK"),GTK_RESPONSE_ACCEPT,("CANCEL"),GTK_RESPONSE_REJECT, NULL); 
    
    // Create a label and attach it to scrolled window, then insert scrolled window into the content area of the dialog
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog)); 
    
    const char * c = directions.c_str(); 
    label = gtk_label_new(c);
    gtk_label_set_justify(GTK_LABEL(label), GTK_JUSTIFY_CENTER);
    gtk_container_add(GTK_CONTAINER(scrolled_window), label); // insert into scrolled window
    gtk_container_add(GTK_CONTAINER(content_area), GTK_WIDGET(scrolled_window)); //insert into dialog box
    
    //resize widget
    gtk_widget_set_size_request(scrolled_window, 600, 400);
    gtk_widget_show_all(dialog); //show popup
    
    // Connecting the "response" signal from the user to the associated callback function
    g_signal_connect(GTK_DIALOG(dialog),"response",G_CALLBACK(on_dialog_response),NULL); 
}

//callback for start intersection searchbar
void nav_entry_1(GtkWidget *widget, ezgl::application *application){
    const gchar *input = gtk_entry_get_text((GtkEntry *) widget);   
    std::string str(input);
    
}
//callback for end intersection callback
void nav_entry_2(GtkWidget *widget, ezgl::application *application){
    const gchar *input = gtk_entry_get_text((GtkEntry *) widget);   
    std::string str(input);
}

// callback function for help button
void display_help(GtkWidget *widget, ezgl::application *application){
        //show help window
        gtk_widget_show_all(GTK_WIDGET(application->get_object("helpDialog"))); //scrolledWindow
}

// function that parses navigation search bar inputs and returns matching intersection
IntersectionIdx findIntersectionNavigate(std::string streetNames, ezgl:: application *application){
    
    sharedIntersectionsNav.clear();
    //remove lowercase and spaces
    std::transform(streetNames.begin(), streetNames.end(), streetNames.begin(), ::tolower); 
    streetNames.erase(std::remove_if(streetNames.begin(), streetNames.end(), isspace), streetNames.end());
    
    //street name variables
    std::string firstName;
    std::string secondName;
    
    //splitting string into street names
    for(int i=0;i< streetNames.length(); i++){
        if(streetNames[i]=='&'){
            firstName= streetNames.substr(0, i);
            secondName= streetNames.substr(i+1, streetNames.length()-1);    
        }   
    }
    
    //if only one word is entered then return
    if(firstName == "" || secondName == ""){
        return -1;
    }
    
    //finding street name matches
    std::vector<StreetIdx> first = findStreetIdsFromPartialStreetName(firstName);
    std::vector<StreetIdx> second = findStreetIdsFromPartialStreetName(secondName);
    
    //finding shared intersections
    for(int i = 0; i < first.size(); i++){
        for(int j = 0; j<second.size(); j++){
            std::pair<StreetIdx,StreetIdx> streetIDs =std::make_pair(first[i],second[j]);
            std::vector<IntersectionIdx> sharedIntersectionsTemp;
            sharedIntersectionsTemp=findIntersectionsOfTwoStreets(streetIDs);
            for (int k = 0; k < sharedIntersectionsTemp.size(); k ++){
                sharedIntersectionsNav.push_back(sharedIntersectionsTemp[k]);
            }
        }
    }
     
    //if no intersections found return
    if(sharedIntersectionsNav.size() == 0){
        return -1;
    }
    
    return sharedIntersectionsNav[0];
    
}

// function that highlights intersections in navigation mode
void highlightIntersectionsNav(double x, double y){
  
  //find closest intersection
  LatLon position = LatLon(convertYtoLatitude(y), convertXtoLongitude(x));
  int intersectionID = findClosestIntersection(position);
  int clickRadius = 10;
  
  //clear path and do not draw if 2 have already been selected, this means path has been chosen and will be reset
  if(sharedIntersectionsNav.size()==2){
      navigationPath.clear();
      sharedIntersectionsNav.clear();
  }

  //check if clicked on intersection or something else
  if((x<=intersections[intersectionID].x+clickRadius)&&(x>=intersections[intersectionID].x-clickRadius)
          &&(y<=intersections[intersectionID].y+clickRadius)&&(y>=intersections[intersectionID].y-clickRadius)){
    
    //if size is one, set showDriections such that on next click directions will popup
    if(sharedIntersectionsNav.size()==1){
        showDirections=true;
    }
    //if size is less than 2, pushback selected intersections
    if(sharedIntersectionsNav.size()<2){
        sharedIntersectionsNav.push_back(intersectionID);
    } 
  } 
}

// function that highlights intersections in navigation mode
void drawIntersectionsNav(ezgl::renderer *g){
    
    int intersectionRad=25;
    ezgl::rectangle visibleWorld = g->get_visible_world();
    
    //get radius based on zoom level
    if(visibleWorld.area()<SMALLESTZOOMARROWS){ 
        intersectionRad=5;
    }
    else if(visibleWorld.area()<SMALLZOOMARROWS){
        intersectionRad=10;
    }   
    else if(visibleWorld.area()<MEDZOOMARROWS){
        intersectionRad=15;
    }
    else if(visibleWorld.area()<LARGEZOOMARROWS){
        intersectionRad=20;
    }
    
    //if not in search mode
    if(!search){
        if(sharedIntersectionsNav.size()>1){
            // find navigation path since 2nd click has occurred and end intersection has been selected
            navigationPath=findPathBetweenIntersections(sharedIntersectionsNav[0], sharedIntersectionsNav[1], TIME_PENALTY);
            start =sharedIntersectionsNav[0];
            end = sharedIntersectionsNav[1];
        }
        if(sharedIntersectionsNav.size()==1){
            //only one intersection has been selected, draw start
            g->set_color(ezgl::GREEN);
            g->fill_arc({convertLongitudetoX(getIntersectionPosition(sharedIntersectionsNav[0]).longitude()),
            convertLatitudetoY(getIntersectionPosition(sharedIntersectionsNav[0]).latitude())}, intersectionRad, 0, 360); 
        } 
    }
}

// function that draws green start intersection for search
void drawSearchStartPoint(ezgl::renderer *g, StreetSegmentIdx prev, StreetSegmentIdx current, int intersectionRad){
    
    StreetSegmentInfo prevSeg= getStreetSegmentInfo(prev);
    StreetSegmentInfo currentSeg= getStreetSegmentInfo(current);
    
    //find shared intersection between first and second segment
    if((currentSeg.to==prevSeg.to)||(currentSeg.from==prevSeg.to)){
        
        //start must be prevSeg.from
        g->set_color(ezgl::GREEN);
        g->fill_arc({convertLongitudetoX(getIntersectionPosition(prevSeg.from).longitude()),
        convertLatitudetoY(getIntersectionPosition(prevSeg.from).latitude())}, intersectionRad, 0, 360);
    }
    else{
        
        //start must be prevSeg.to
        g->set_color(ezgl::GREEN);
        g->fill_arc({convertLongitudetoX(getIntersectionPosition(prevSeg.to).longitude()),
        convertLatitudetoY(getIntersectionPosition(prevSeg.to).latitude())}, intersectionRad, 0, 360);
    
    }
}


void drawSearchEndPoint(ezgl::renderer *g, StreetSegmentIdx prev, StreetSegmentIdx current, int intersectionRad){
    
    StreetSegmentInfo prevSeg= getStreetSegmentInfo(prev);
    StreetSegmentInfo currentSeg= getStreetSegmentInfo(current);
    
    //find shared intersection between last and second last segment
    if((currentSeg.to==prevSeg.to)||(currentSeg.to==prevSeg.from)){
        
        //end must be currentSeg.from
        g->set_color(ezgl::RED);
        g->fill_arc({convertLongitudetoX(getIntersectionPosition(currentSeg.from).longitude()),
        convertLatitudetoY(getIntersectionPosition(currentSeg.from).latitude())}, intersectionRad, 0, 360);
    }
    else{
        
        //end must be currentSeg.to
        g->set_color(ezgl::RED);
        g->fill_arc({convertLongitudetoX(getIntersectionPosition(currentSeg.to).longitude()),
        convertLatitudetoY(getIntersectionPosition(currentSeg.to).latitude())}, intersectionRad, 0, 360);   
    }
}
