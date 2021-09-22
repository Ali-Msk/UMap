/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   m3_internal.h
 * Author: wangh365
 *
 * Created on March 28, 2021, 11:10 AM
 */
#pragma once //protects against multiple inclusions of this header file

#include "m3.h"
#include "m2_internal.h"
#include <queue>
#include <algorithm>

//struct for intersectionNode
struct IntersectionNode{
    int visited;
    int closed; 
    IntersectionIdx previousIntersection;
    double travelTime;
    double heuristic;
    StreetSegmentIdx previousSegment;
    IntersectionNode(){
        visited = 0;
        closed = 0; 
        previousIntersection = -1; 
        previousSegment = -1;
        travelTime = std::numeric_limits<int>::max();
        heuristic = std::numeric_limits<int>::max();
    }
    
};

extern bool showNavigation; //descriptions in m3.cpp
extern double TIME_PENALTY;
extern std::vector<StreetSegmentIdx> navigationPath;
extern std::string directions;
extern std::vector<IntersectionIdx> sharedIntersectionsNav; //shared intersections from find feature data
extern IntersectionIdx start;
extern IntersectionIdx end;

double findHeuristic(int currentIntersection, int endIntersection); //find the heuristic value of the current intersection
double findIntersectionTraveltime(int startIntersection, int endIntersection, int prevSegment);
int legalPath(int startIntersection, int endIntersection); //check if the direction is legal
std::vector<StreetSegmentIdx> backTrack(std::vector<IntersectionNode> &intersectionNodes, IntersectionIdx destination); //reconstruct the path
void displayDirections(ezgl::renderer *g, std::vector<StreetIdx> path); // function that displays path and directions string
StreetSegmentIdx findStreetSegment(int firstIntersection, int secondIntersection, StreetSegmentIdx previousSegment);
bool checkTurn(IntersectionIdx current, IntersectionIdx next, IntersectionIdx prev);
void navigate_map(GtkWidget *widget, ezgl::application *application);           //callback for navigation mode button
void nav_entry_1(GtkWidget *widget, ezgl::application *application);            //callback for start intersection searchbar
void nav_entry_2(GtkWidget *widget, ezgl::application *application);            //callback for end intersection callback
void getDirections(ezgl:: renderer *g, std::vector<StreetSegmentIdx> path);     // function that gets directions and appends them to a string
void on_dialog_response(GtkWidget *widget, ezgl::application *application);     // callback function for directions dialog window response signal
void popupDirections(ezgl::application *application);                           // function that creates dialog window with directions
void appendStraightDistance(StreetSegmentInfo prev, int segmentPathLength);     // function that adds the direction continue straigt to directions string
double getCrossProductAngle(StreetSegmentInfo prev, StreetSegmentInfo current,  // function that gets cross product angle by using proper vector based
        StreetSegmentIdx prevIdx, StreetSegmentIdx currentIdx);                 // on street segment from and to, and curve points
void drawDirectionArrows(ezgl::renderer *g, StreetSegmentIdx current,           //function that draws all direction arrows based on zoom level
        StreetSegmentIdx prev, int arrowLen); 
void drawArrow(ezgl::renderer *g, StreetSegmentIdx prev, IntersectionIdx        //function that draws individual direction arrow
segStart, IntersectionIdx segEnd, int curveStart, int curveEnd, int firstCurve, int numCurve, int arrowLen);
void appendTurnDistance(std::string streetName, double crossProductAngle);      //function that adds turn direction to directions string
void appendStart(std::string startName);
double calculateCrossProductAngle(double Bx, double By, double Ax, double Ay,   //function that calculates cross product angle
        double Cx, double Cy);
double getCrossProductAngleWithCorrectPoints(IntersectionIdx sharedInt,         //function that determines the correct vector to pass in for cross product
        IntersectionIdx startPrev, IntersectionIdx endCurr, int 
        firstCurveCurrent, int lastCurvePrev, StreetSegmentInfo prev, 
        StreetSegmentInfo current,StreetSegmentIdx prevIdx, StreetSegmentIdx 
        currentIdx); 
IntersectionIdx findIntersectionNavigate(std::string streetNames, ezgl::        // function that parses navigation search bar inputs
                application *application);
void drawIntersectionsNav(ezgl::renderer *g);                                   // function that parses sarch bar and finds matching intersection to names
void highlightIntersectionsNav(double x, double y);                             // function that highlights intersections in navigation mode
void drawSearchStartPoint(ezgl::renderer *g, StreetSegmentIdx prev, StreetSegmentIdx current, int intersectionRad); // function that draws green start intersection for search
void drawSearchEndPoint(ezgl::renderer *g, StreetSegmentIdx prev, StreetSegmentIdx current, int intersectionRad); // function that draws red end intersection for search
void on_help_response(GtkWidget *widget, ezgl::application *application); // call back function for help window response
void display_help(GtkWidget *widget, ezgl::application *application); // call back function for help button