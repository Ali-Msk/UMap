#pragma once //protects against multiple inclusions of this header file

#include <string> 
#include <vector> 
#include "StreetsDatabaseAPI.h" 
#include <iostream> 
#include "m2.h" 
#include "StreetsDatabaseAPI.h"  
#include "ezgl/application.hpp" 
#include "ezgl/graphics.hpp" 
#include "m1.h" 
#include "OSMDatabaseAPI.h" 
#include "m3_internal.h"

extern std::unordered_map<StreetIdx, std::vector<StreetSegmentIdx>> streetSegmentsOfStreet;
//declaring struct for intersection data
struct intersection_data {
    double x=-1; 
    double y; 
    std::string name; 
    bool highlight = false; 
};

//declaring struct for feature data
struct feature_data {
    FeatureType featureType;
    int numPoints;
    std::vector <LatLon> points;
    std::string name;
    bool highlight = false;
    int prevHighlighted=-1;
};

//declaring struct for POI data
struct pointOfInterest_data {
    std::string name;
    std::string type;
    double y=-1;
    double x;
    OSMID osmid;
    bool highlight = false;
};

//declaring struct for transit data
struct transitData{
    std::string type;
    double x;
    double y;
    std::string name;
    std::string stationName;
    std::string color;
};

extern std::vector<intersection_data> intersections; //declaring intersections info vector
extern bool showDirections;
extern bool search;
//function prototypes for conversions and info collection
double convertLongitudetoX(double longitude); //function that converts longtitude to x coordinate
double convertLatitudetoY(double latitude); //function that converts latitude to y coordinate
double convertYtoLatitude(double y); //function that converts latitude to y coordinate
double convertXtoLongitude(double x); //function that converts longitude to x coordinate
int findClosestPOI(double x, double y); //function that finds closest POI

//function prototypes for drawing
void act_on_mouse_click(ezgl::application *application, GdkEventButton *event, double x, double y); //callback mouseclick function
void draw_main_canvas (ezgl::renderer *g); //draws main canvas
void drawStreets(ezgl::renderer *g); //function to help draw streets
void drawIntersections(ezgl::renderer *g); //function to draw intersections
void drawFeatures(ezgl::renderer *g); //function to draw features
void drawScale(ezgl::renderer *g); //function to draw scale
int streetSize(StreetSegmentInfo segmentInfo); //determine the size of street segment, 1 if small, 2 if long
void displayStreets(ezgl::renderer *g, int i); //function that helps display actual street segment
void displayStreetNames(ezgl::renderer *g, StreetSegmentInfo streetInfo,int segmentId); //function that displays street names
void drawPointsOfInterests(ezgl::renderer *g); //function that displays POI
ezgl::color invertedColor(ezgl::color color);
void setOriginalColors(); //function that sets original color
void setColorBlindColors(); //function that sets color blind colors
void setNightModeColors(); //function that sets night mode
void highlightIntersections(double x, double y);//function that highlights intersections
void highlightPOI(double x, double y);//functions that highlights POI
void findIntersections(std::string streetNames,ezgl:: application *application); //function that highlights intersections as specified by find
void drawSubwayStations(ezgl::renderer *g); //function that draws subway stations
void drawImportantFeatures(ezgl::renderer *g); //function that drawa prioritized features first
void drawSearchedIntersections(ezgl::renderer *g); //function that draws intersections from find featuree
void importImages(); //function to import images
void loadDataStructures(); //function to load data structures

//load OSM data
void loadOSMData(const std:: string); 

//functions for making new map
void initalizeNewMap(ezgl::application *application); 

//callback functions 
void search_button(GtkWidget *widget, ezgl::application *application);
void initial_setup(ezgl::application *application, bool new_window);
void text_entry(GtkWidget *widget, ezgl::application *application);
void clicked_search_bar(GtkWidget *widget, ezgl::application *application);
void display_subway_stations(GtkWidget *widget, ezgl::application *application);

//buttons not created with glade
void colour_blind_mode(GtkWidget *widget, ezgl::application *application);   
void night_mode(GtkWidget *widget, ezgl::application *application);  
void changeMap(GtkWidget *widget, ezgl:: application *application);  

void freeDataStructures();  
void closeAll(); 