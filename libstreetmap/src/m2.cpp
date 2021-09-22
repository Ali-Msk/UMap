
#include <iostream>
#include "m2.h"
#include <iostream>
#include "StreetsDatabaseAPI.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1.h"
#include "OSMDatabaseAPI.h"
#include "m2_internal.h"
#include <cmath>

//variable declarations
double minX=std::numeric_limits<int>::max();
double minY=std::numeric_limits<int>::max();
double maxX=std::numeric_limits<int>::min();
double maxY=std::numeric_limits<int>::min();

//custom colours for aesthetics
static constexpr ezgl::color LakeBlue(0x80, 0xB7, 0xFF);
static constexpr ezgl::color StreamBlue(0x73, 0xB0, 0xFF);
static constexpr ezgl::color GreenSpace(0x99, 0xD1, 0x81);
static constexpr ezgl::color background(0xFC, 0xF4, 0XE4);
static constexpr ezgl::color street(0xDE, 0xD2, 0xBC);    //custom street colour
static constexpr ezgl::color border(0xD9, 0xCB, 0xB1); //cdb26c
static constexpr ezgl::color highway(0xEE, 0xCC, 0x11); 
static constexpr ezgl::color majorStreets(0xD6, 0xBE, 0x91); 
static constexpr ezgl::color streetsNames(0x44, 0x27, 0x00); 
static constexpr ezgl::color infoBox(0xDA, 0xCD, 0xB4); //d6c086


//color blind constants 
static constexpr ezgl::color LakeBlueColorBlind(0x6F, 0x69, 0xE5);
static constexpr ezgl::color StreamBlueColorBlind(0x73, 0xB0, 0xFF);
static constexpr ezgl::color GreenSpaceColorBlind(0xAB, 0xAF, 0x14);
static constexpr ezgl::color backgroundColorBlind(0xFF, 0xFF, 0XFF);
static constexpr ezgl::color streetColorBlind(0xDE, 0xD2, 0xBC);    //custom street colour
static constexpr ezgl::color borderColorBlind(0xB2, 0xA7, 0x3D); //b2a73d
static constexpr ezgl::color highwayColorBlind(0x92, 0x8A, 0x00); 
static constexpr ezgl::color majorStreetsColorBlind(0xBC, 0xB1, 0x00);
static constexpr ezgl::color infoBoxColorBlind(0xBE, 0xB2, 0x41); //beb241
//night mode constants
static constexpr ezgl::color LakeBlueNight(0x00, 0x3C, 0x74); 
static constexpr ezgl::color StreamBlueNight(0x12, 0x24, 0x9A);  
static constexpr ezgl::color GreenSpaceNight(0x00, 0x57, 0x03); //005703
static constexpr ezgl::color backgroundNight(0x19, 0x1B, 0X61); 
static constexpr ezgl::color streetNight(0x62, 0x61, 0x5D);    //custom street colour
static constexpr ezgl::color borderNight(0x10, 0x06, 0x50); //100650
static constexpr ezgl::color highwayNight(0x42, 0x3C, 0x45);
static constexpr ezgl::color majorStreetsNight(0x4D, 0x3B, 0x74);
static constexpr ezgl::color islandNight(0x16, 0x0A, 0x60);
static constexpr ezgl::color infoBoxNight(0x14, 0x08, 0x66); //140866

//declaring data structures
std::unordered_map <OSMID, std::string> streetSegmentTypes; //map for highway data
std::vector <intersection_data> intersections; //declaring intersections info vector
std::vector <feature_data> features; //declaring  feature info vector
std::vector <pointOfInterest_data> pointsOfInterest; //declaring POI info structure
std::unordered_map <const OSMNode*, transitData> transitStations; //transit stations data
std::vector <const OSMRelation*> subwayRoutes; //subway routes data
std::vector<IntersectionIdx> sharedIntersections; //shared intersections from find feature data
std::unordered_map<StreetIdx, std::vector<StreetSegmentIdx>> streetSegmentsOfStreet;

//Colors in use at the moment.
//these are not constats and change based on viewing mode (colorblind or night mode)
ezgl::color unknownColor;
ezgl::color parkColor;
ezgl::color breachColor ;
ezgl::color lakeColor;
ezgl::color riverColor ;
ezgl::color buildingColor;
ezgl::color islandColor ;
ezgl::color greenspaceColor;
ezgl::color golfcourseColor;
ezgl::color streamColor;
ezgl::color backgroundColor;
ezgl::color streetColor;
ezgl::color borderColor;
ezgl::color POIColor;
ezgl::color highwayColor;
ezgl::color majorStreetsColor;
ezgl::color scaleColor;
ezgl::color subwayNameColor;
ezgl::color streetNamesColor;
ezgl::color infoBoxColor;

//setting booleans for color mode
bool nightMode = false;
bool colorBlindMode = false;
bool displaySubwayStations = false;
bool showDirections = false;
bool search=false;

//constants for streets
const double MINZOOMSMALLSTREET = 50000;
const double MINZOOMMEDIUMSTREET = 50000000;
const double MINZOOMBIGSTREET = 700000000;
const double SMALLROADLENGTH = 10000;
const double MEDIUMROADLENGTH = 15000;

//declare arrow image
std:: string displayOneWayStreet(ezgl::renderer *g, StreetSegmentInfo street, double x1, double y1, double x2, double y2);
ezgl::surface *smallArrowImage;
ezgl::surface *POIIconRed;
ezgl::surface *POIIconBlue;
ezgl::surface *POIIconGreen;
ezgl::surface *greenSubway;
ezgl::surface *yellowSubway;
ezgl::surface *blueSubway;
ezgl::surface *redSubway;

//function to draw map
void drawMap(){
    //setting up ezgl graphics
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    ezgl::application application(settings);
    
    importImages();//import images
    setOriginalColors(); //sets up original colors
    
   //finding bounds of city
    for (int i=0;i< getNumIntersections(); i++){
        
        //iterate and find maximum and minimum coordinates
        maxX = std::max(maxX, getIntersectionPosition(i).longitude());
        minX = std::min(minX, getIntersectionPosition(i).longitude());
        maxY = std::max(maxY, getIntersectionPosition(i).latitude());
        minY = std::min(minY, getIntersectionPosition(i).latitude());              
    }
    
    loadDataStructures(); //load data structures  
    
    std::string map="/cad2/ece297s/public/maps/toronto_canada.osm.bin";
    loadOSMData(map);

    
    
     //drawing main canvas
     ezgl::rectangle initial_world({convertLongitudetoX(minX), convertLatitudetoY(minY)},
                {convertLongitudetoX(maxX), convertLatitudetoY(maxY)});
     application.add_canvas("MainCanvas", draw_main_canvas, initial_world, backgroundColor);  

     application.run(initial_setup, act_on_mouse_click , nullptr, nullptr);  


}
//function to import images
void importImages(){
    //import images
    //https://www.cleanpng.com/png-rapid-transit-train-station-computer-icons-subway-608926/ source for subway icons
    //https://www.vectorstock.com/royalty-free-vector/map-pin-icon-vector-13419750 source for POI icons
    smallArrowImage = ezgl::renderer::load_png("libstreetmap/resources/arrow.png");
    POIIconGreen = ezgl::renderer::load_png("libstreetmap/resources/green-pin-icon.png");
    POIIconBlue = ezgl::renderer::load_png("libstreetmap/resources/blue-pin-icon.png");
    POIIconRed = ezgl::renderer::load_png("libstreetmap/resources/orange-pin-icon.png");
    greenSubway = ezgl::renderer::load_png("libstreetmap/resources/green.png"); ;
    yellowSubway = ezgl::renderer::load_png("libstreetmap/resources/yellow.png"); ;
    blueSubway = ezgl::renderer::load_png("libstreetmap/resources/blue.png");;
    redSubway = ezgl::renderer::load_png("libstreetmap/resources/red.png");;
}
//function to load data structures
void loadDataStructures(){
    
    //storing intersection data in a vector 
    //reserving an extra item to store id of previously highlighted intersections
    intersections.resize(getNumIntersections()+1);
    for (int k=0;k< getNumIntersections(); k++){
        
        intersections[k].name=getIntersectionName(k);
        intersections[k].y=convertLatitudetoY(getIntersectionPosition(k).latitude());
        intersections[k].x=convertLongitudetoX(getIntersectionPosition(k).longitude());
        
    }
    //storing feature data in a map    
    features.resize(getNumFeatures()+1);
    for (int i=0;i< getNumFeatures(); i++){
        
        features[i].name = getFeatureName(i);
        features[i].numPoints = getNumFeaturePoints(i);
        features[i].featureType = getFeatureType(i);
        features[i].highlight = false;
        features[i].prevHighlighted = false;
        features[i].points.resize(features[i].numPoints);
        for (int j = 0; j < features[i].numPoints; j++){
            features[i].points[j] = getFeaturePoint(i, j);
        }
        
    }
    //storing poi data in a map
    pointsOfInterest.resize(getNumPointsOfInterest()+1);
    for (int i=0;i< getNumPointsOfInterest(); i++){
        pointsOfInterest[i].type = getPOIType(i);
        pointsOfInterest[i].name = getPOIName(i);
        pointsOfInterest[i].y=convertLatitudetoY(getPOIPosition(i).latitude());
        pointsOfInterest[i].x=convertLongitudetoX(getPOIPosition(i).longitude());
        pointsOfInterest[i].osmid = getPOIOSMNodeID(i);
    } 
    
    for(int i = 0; i < getNumStreetSegments(); i++){
        StreetSegmentInfo tempInfo = getStreetSegmentInfo(i);
        streetSegmentsOfStreet[tempInfo.streetID].push_back(i);
    }
}
int count = 0;
std::vector<StreetIdx> test ;
//function to draw main canvas
void draw_main_canvas (ezgl::renderer *g) {
    count ++;
    //setting visible world
    ezgl::rectangle visibleWorld= g->get_visible_world();
    g->set_color(backgroundColor);
    g->fill_rectangle(visibleWorld);
    
    //drawing map items
    drawImportantFeatures(g);
    drawFeatures(g);
    drawStreets(g);
    if(showNavigation){
        drawIntersectionsNav(g);
        displayDirections(g, navigationPath);        
    }
    else{
        drawIntersections(g);
    }
    drawPointsOfInterests(g);
    drawScale(g);      
    if(displaySubwayStations){
        drawSubwayStations(g);
    }
}
//function to draw intersections
void drawIntersections(ezgl::renderer *g){
    //keep track of how zoomed in the user is
   ezgl::rectangle visibleWorld= g->get_visible_world();
   int minZoomBigRad = 5000000;
   int minZoomSmallRad = 775000;
   double intersectionRad=10;
   
   //if shared intersections is not empty display intersections from find feature
   if(sharedIntersections.size()!=0){
       for(int i = 0; i<sharedIntersections.size(); i++){
            double x = intersections[sharedIntersections[i]].x; 
            double y = intersections[sharedIntersections[i]].y;

            g->set_color(ezgl::CORAL);
            //drawing circles centered at intersection
            g->fill_arc({x,y}, intersectionRad, 0, 360);
            intersections[intersections[intersections.size()-1].y].highlight=false; //unhighlight other highlighted intersection
       }
   }
   //check if zoomed in enough to display intersections
   if(visibleWorld.area()<(minZoomBigRad)){   
    //draw intersections
       if(visibleWorld.area()<minZoomSmallRad){
          intersectionRad=5;
       }
        g->set_coordinate_system(ezgl::WORLD);
        //if reserved map element for highlighted data's y member is not -1, this means an intersection has been highlighted
        if(intersections[intersections.size()-1].y!=-1) {
            //double check if still highlighted
            if(intersections[intersections[intersections.size()-1].y].highlight){
                //get position
                double x = intersections[intersections[intersections.size()-1].y].x; 
                double y = intersections[intersections[intersections.size()-1].y].y;
                g->set_color(ezgl::CORAL);
                //drawing circles centered at intersection
                g->fill_arc({x,y}, intersectionRad, 0, 360);
                intersections[intersections.size()-1].name=intersections[intersections[intersections.size()-1].y].name;

                //drawing info box
                g->set_color(infoBoxColor);
                g->set_coordinate_system(ezgl::SCREEN);
                ezgl::rectangle visibleScreen = g->get_visible_screen();
                g->fill_rectangle({visibleScreen.left(),visibleScreen.top()}, {visibleScreen.right(), visibleScreen.top()-50});
                g->set_color(borderColor);
                g->draw_rectangle({visibleScreen.left(),visibleScreen.top()}, {visibleScreen.right(), visibleScreen.top()-50});
                //print text
                g->set_color(scaleColor);
                g->set_text_rotation(0);
                g->set_font_size(14);
                g->draw_text({(visibleScreen.left()+visibleScreen.right())/2, visibleScreen.top()-25}, intersections[intersections.size()-1].name);  
            }
        }
   }
   //if not zoomed enough unhighlight intersection
   else{
       intersections[intersections[intersections.size()-1].y].highlight=false;
   }
}
//function to draw POI
void drawPointsOfInterests(ezgl::renderer *g){
    double minZoomPOI = 1000000;
    ezgl::rectangle visibleWorld= g->get_visible_world();
    //if zoomed in enough, draw POI
    if(visibleWorld.area()<minZoomPOI){
        //iterate through all POI
        for (int k=0;k<pointsOfInterest.size()-2; k++){
            //get location
            double x =pointsOfInterest[k].x; 
            double y = pointsOfInterest[k].y;

            g->set_coordinate_system(ezgl::WORLD);
            //if not highlighted draw as blue
            if(!pointsOfInterest[k].highlight) {
                g->draw_surface(POIIconBlue,{x, y});
            }
            else{ //if highlighted draw orange
                pointsOfInterest[pointsOfInterest.size()-1].name=pointsOfInterest[k].name;
                g->draw_surface(POIIconRed,{x, y});               
            }
        }
        //if reserved map element for previously highlighted data indicates a POI is highlighted, draw info box
        if(pointsOfInterest[pointsOfInterest.size()-1].y!=-1 && pointsOfInterest[pointsOfInterest[pointsOfInterest.size()-1].y].highlight!=false){
            g->set_color(infoBoxColor);
            g->set_coordinate_system(ezgl::SCREEN);
            ezgl::rectangle visibleScreen = g->get_visible_screen();
            g->fill_rectangle({visibleScreen.left(),visibleScreen.top()}, {visibleScreen.right(), visibleScreen.top()-50});
            g->set_color(borderColor);
            g->draw_rectangle({visibleScreen.left(),visibleScreen.top()}, {visibleScreen.right(), visibleScreen.top()-50});
            //print text
            g->set_color(scaleColor);
            g->set_text_rotation(0);
            g->set_font_size(14);
            g->draw_text({(visibleScreen.left()+visibleScreen.right())/2, visibleScreen.top()-25}, pointsOfInterest[pointsOfInterest.size()-1].name);  
        }      
    }
    else{ //if not zoomed in enough unhighlight
        if(pointsOfInterest[pointsOfInterest.size()-1].y!=-1){
            pointsOfInterest[pointsOfInterest[pointsOfInterest.size()-1].y].highlight=false;
        }
    }
}

//function to display streets
void drawStreets(ezgl::renderer *g){

   //only draw if its in range
   for(int i = 0; i < getNumStreetSegments(); i++){
        StreetSegmentInfo streetInfo = getStreetSegmentInfo(i);
        OSMID id = streetInfo.wayOSMID;
        //set basic color and determine x and y coordinate of streets 
        LatLon fromCoordinates = getIntersectionPosition(streetInfo.from);
        LatLon toCoordinates = getIntersectionPosition(streetInfo.to);
        double x1 = convertLongitudetoX(toCoordinates.longitude());
        double x2 = convertLongitudetoX(fromCoordinates.longitude());
        double y1 = convertLatitudetoY(toCoordinates.latitude());
        double y2 = convertLatitudetoY(fromCoordinates.latitude());
        
        //determine visible world in order to not draw streets that are not currently displayed on the streets 
        ezgl::rectangle visibleWorld= g->get_visible_world();
        ezgl::rectangle doubleVisibleWorld;
        int areaToSwitchToCustomLoadScreen = 50000;
        
        if(visibleWorld.area() > areaToSwitchToCustomLoadScreen){
            doubleVisibleWorld = visibleWorld;
        }else{
            ezgl::rectangle tempVisibleWorld({visibleWorld.bottom_left().x - std::sqrt(visibleWorld.area()) * 3, visibleWorld.bottom_left().y - std::sqrt(visibleWorld.area()) *3},
                  visibleWorld.width()*9, visibleWorld.height()*9);
            doubleVisibleWorld = tempVisibleWorld;
        }
        
        //boolean to determine if display is required
        bool display = true;
        if(!doubleVisibleWorld.contains(x1, y1) && !doubleVisibleWorld.contains(x2, y2)){
            display = false;
        }
        int smallStreetWidth = 5;
        int mediumStreetWidth = 4 ;
        if(display){
            //if zoomed in show all types of streets      
            if(visibleWorld.area() < MINZOOMSMALLSTREET){
                 if(streetSegmentTypes[id] == "highway"  || streetSegmentTypes[id] == "long"){ //major roads
                    g->set_color(highwayColor); //highwayColor
                    g->set_line_width(smallStreetWidth);
                    displayStreets(g,i);    
                }else if(streetSegmentTypes[id] == "highway-link" ){ //set links to highway yellow
                    g->set_color(highwayColor); //major streetColor;
                    g->set_line_width(smallStreetWidth);
                    displayStreets(g,i); 
                }else if((streetSize(streetInfo) == 2) || streetSize(streetInfo) == 3){ //smaller streets
                    g->set_color(majorStreetsColor); //major streetColor;
                    displayStreets(g,i); 
                    g->set_line_width(smallStreetWidth);
                } else {
                    g->set_color(streetColor); //residential streets
                    g->set_line_width(smallStreetWidth);
                    displayStreets(g,i);
                }   
                displayStreetNames(g, streetInfo,i); //display street names
            }else if(visibleWorld.area() < MINZOOMMEDIUMSTREET){
                //set line width to thinner
              if(streetSegmentTypes[id] == "highway"  || streetSegmentTypes[id] == "long"){ 
                    g->set_color(highwayColor); //highwayColor
                    g->set_line_width(mediumStreetWidth); //set line width to thinner
                    displayStreets(g,i);    
                }else if(streetSegmentTypes[id] == "highway-link" ){
                    g->set_color(highwayColor); //major streetColor;
                    g->set_line_width(mediumStreetWidth);
                    displayStreets(g,i); 
                }else if((streetSize(streetInfo) == 2) || streetSize(streetInfo) == 3){
                    g->set_color(majorStreetsColor); //major streetColor;
                    displayStreets(g,i); 
                    g->set_line_width(mediumStreetWidth);
                } else {
                    g->set_color(streetColor); //small streetColor;
                    g->set_line_width(mediumStreetWidth);
                    displayStreets(g,i);
                }              
                displayStreetNames(g, streetInfo,i);             
            }else if(visibleWorld.area() < MINZOOMBIGSTREET){   
                if(streetSegmentTypes[id] == "highway" || streetSegmentTypes[id] == "long"){
                    g->set_color(highwayColor); //highwayColor
                    g->set_line_width(3);
                    displayStreets(g,i);    
                }else if(streetSegmentTypes[id] == "highway-link" ){
                    g->set_color(highwayColor); //major streetColor;
                    g->set_line_width(1);
                    displayStreets(g,i); 
                }else if((streetSize(streetInfo) == 2) || streetSize(streetInfo) == 3){
                    g->set_color(majorStreetsColor); //major streetColor;
                    displayStreets(g,i); 
                    g->set_line_width(3);
                }  else {
                    g->set_color(streetColor); //small streetColor;
                    g->set_line_width(1);
                    displayStreets(g,i);
                }              
                displayStreetNames(g, streetInfo,i); //display names
            }else{         
                if(streetSegmentTypes[id] == "highway" || streetSegmentTypes[id] == "long" ){
                    g->set_line_width(5);
                    g->set_color(highwayColor); //highwayColor
                    displayStreets(g,i);   
                }else if(streetSize(streetInfo) == 3){
                    g->set_color(majorStreetsColor); //major streetColor;
                    g->set_line_width(3);
                    displayStreets(g,i); 
                }
            }
        }

   }
}
//function to display streets
void displayStreets(ezgl::renderer *g, int i){
     //getting coordinates of each street segment
     StreetSegmentInfo streetInfo = getStreetSegmentInfo(i);
     int numCurvePoints = streetInfo.numCurvePoints;
     LatLon fromCoordinates = getIntersectionPosition(streetInfo.from);
     LatLon toCoordinates = getIntersectionPosition(streetInfo.to);
     
    //set line style
    g->set_line_cap(ezgl::line_cap::round); // Butt ends
    g->set_line_dash(ezgl::line_dash::none); // Solid line
     
    //display street names
    for(int j= 0; j<numCurvePoints; j++){
        //get the lat lon of the curve points 
        LatLon curvePointCoordinates = getStreetSegmentCurvePoint(i,j);  
        g->draw_line({convertLongitudetoX(fromCoordinates.longitude()), convertLatitudetoY(fromCoordinates.latitude())}, {convertLongitudetoX(curvePointCoordinates.longitude()), convertLatitudetoY(curvePointCoordinates.latitude())});
        //update from Coordinates
        fromCoordinates = curvePointCoordinates;
    }    
    g->draw_line({convertLongitudetoX(fromCoordinates.longitude()), convertLatitudetoY(fromCoordinates.latitude())}, {convertLongitudetoX(toCoordinates.longitude()), convertLatitudetoY(toCoordinates.latitude())});   

}
//function to display one way streets
std:: string displayOneWayStreet(ezgl::renderer *g, StreetSegmentInfo streetInfo, double x1, double y1, double x2, double y2){
    bool oneWay = streetInfo.oneWay;
    std:: string name = getStreetName(streetInfo.streetID);
    
    //dont display one ways for unknown streets
    if(name == "<unknown>") return "";
    
    if(!oneWay){
        return name;
    }
    
    //add arrows to street names based on the direction
    if(x2 > x1 && y2 != y1){
        name = " -> " + name + " -> "; 
    }else if(x1>x2 && y1 != y2){
        name = " <- " + name + " <- ";
    }else{
        if(y2 > y1){
             name = " <- " + name + " <- ";
        }else{
            name = " -> " + name + " -> ";
        }
    }
    
    return name;

    
  
}

//display names on streets on proper street
void displayStreetNames(ezgl::renderer *g, StreetSegmentInfo streetInfo, int segmentId){
    
    //determine from and to coordinates of the intersection
    LatLon fromCoordinates = getIntersectionPosition(streetInfo.from);
    LatLon toCoordinates = getIntersectionPosition(streetInfo.to);
    //get location and angle
    double x1 = convertLongitudetoX(fromCoordinates.longitude());;
    double x2 = convertLongitudetoX(toCoordinates.longitude());
    double y1 = convertLatitudetoY(fromCoordinates.latitude());
    double y2 = convertLatitudetoY(toCoordinates.latitude());
    double x = (x1+x2)/2;
    double y = (y1+y2)/2;
    double angle = std::atan2(y2-y1,x2-x1) / kDegreeToRadian;
    
    //if rotated, add back 180 to prevent over rotation
    if(x2-x1 < 0){ 
       angle += 180;
    }
    
    //differernt calucations of where to put street names if no curve points
    if(streetInfo.numCurvePoints != 0){
        LatLon curvePointCoordinates = getStreetSegmentCurvePoint(segmentId,streetInfo.numCurvePoints/2);
        x = convertLongitudetoX(curvePointCoordinates.longitude());
        y = convertLongitudetoX(curvePointCoordinates.latitude());
        angle = std::atan2(x,y) / kDegreeToRadian;
    }
 
    g->set_text_rotation(angle);
   
    std:: string name = displayOneWayStreet(g, streetInfo, x1, y1,x2,y2);
    if(name == "<unknown>") return;
    g->set_font_size(12);
    g->set_color(streetNamesColor);
    double width = findStreetSegmentLength(segmentId);
    g->draw_text({x,y}, name, width, 500000);
}

//determine the size of street segment, if small, 2 if long
int streetSize(StreetSegmentInfo segmentInfo){
    int length = findStreetLength(segmentInfo.streetID);
    if(length <SMALLROADLENGTH || getStreetName(segmentInfo.streetID) == "<unknown>" ){
        return 1;
    }else if(length < MEDIUMROADLENGTH){
        return 2;
    }else{
        return 3;
    }
}
//function to draw beaches, lakes and islands as they must come before other features
void drawImportantFeatures(ezgl::renderer *g){
    bool display  = true;    
     //keep track of how zoomed in the user is
    ezgl::rectangle visibleWorld= g->get_visible_world();
          
     //run through features 
    for (int k=0;k<features.size()-2; k++){
        display = false;
                
        //set color based on type
        switch(features[k].featureType){
            case BEACH    : g->set_color(breachColor);display = true;  break;
            case LAKE    : g->set_color(lakeColor);display = true;  break;
            case ISLAND   : g->set_color(islandColor); display = true; break;
            default: break;
        }
         
        if(display){
            bool inScreen = false;
            std::vector <ezgl::point2d> newPoints;
            //gets the points to pass into draw polly
            for (int i=0;i<features[k].numPoints; i++){
                double x = convertLongitudetoX(features[k].points[i].longitude());
                double y = convertLatitudetoY(features[k].points[i].latitude());
                newPoints.push_back(ezgl::point2d{x, y});
                //checks if the feature is in the screen, otherwise doesnt display 
                if(!inScreen && visibleWorld.contains(x, y)){
                    inScreen = true;
                }
            }
            //if feature is larger than screen, display nonetheless
            if(findFeatureArea(k) > visibleWorld.area()){
                inScreen = true;
            }
            if(!inScreen){
                display = false;
            }
            if(newPoints.size() > 1 ){
                if(display){
                    //draw the feature 
                    g->fill_poly(newPoints);
                }
            }
        }
    }
}
//draws the rest of the features.
void drawFeatures(ezgl::renderer *g){

    bool display  = true;
    bool stream = false;
    
     //keep track of how zoomed in the user i 
    ezgl::rectangle visibleWorld= g->get_visible_world();
    
    //constants for showing based on zoom 
    int minZoomBuildings = 1000000; 
    int minZoomStreams = 100000000; 
          
     //drawing features 
    for (int k=0;k<features.size()-2; k++){
        display = true;
        
        //check for special case stream10s.
        stream = false;
                
        //colors
        switch(features[k].featureType){
            case UNKNOWN : g->set_color(unknownColor);break;
            case PARK : g->set_color(parkColor); break;
            case BEACH : g->set_color(breachColor); break;
            case LAKE : g->set_color(lakeColor); display = false;break;
            case RIVER    : g->set_color(riverColor);display = false; break;
            case ISLAND    : g->set_color(islandColor); display = false;break;
            case BUILDING    : g->set_color(buildingColor);
            
                if(visibleWorld.area()>minZoomBuildings){ //only show past certain zoom 
                    display = false;
                }

                break;
            case GREENSPACE    : g->set_color(greenspaceColor); break;
            case GOLFCOURSE    : g->set_color(golfcourseColor); break;
            case STREAM        : g->set_color(streamColor); 
                stream = true;
                if(visibleWorld.area()>minZoomStreams){ //only show past certain zoom
                    display = false;
                }
                break;
            default: break;
        }
               
        if(display){
            bool inScreen = false;
            std::vector <ezgl::point2d> newPoints;
            //calculate points to pass into draw polly as vector
            for (int i=0;i<features[k].numPoints; i++){
                double x = convertLongitudetoX(features[k].points[i].longitude());
                double y = convertLatitudetoY(features[k].points[i].latitude());
                newPoints.push_back(ezgl::point2d{x, y});
                if(!inScreen && visibleWorld.contains(x, y)){ //check if in screen
                    inScreen = true;
                }
            }
            if(findFeatureArea(k) > visibleWorld.area() ){
                inScreen = true;
            }
            if(!inScreen){
                display = false;
            }
            if(newPoints.size() > 1 ){
                if(display){
                    if(stream){ //if stream, draw line not poly
                        for(int j= 0; j< newPoints.size()-1; j++){
                            g->draw_line(newPoints[j], newPoints[j+1]);
                        }
                    }else{
                        g->fill_poly(newPoints);
                    }
                }
            }
        }
    }
}

//function that converts longitude to x
double convertLongitudetoX(double longitude){
    
    double latAvg = kDegreeToRadian*((maxY + minY)/2.0);
    double x = kEarthRadiusInMeters * longitude * kDegreeToRadian * std::cos(latAvg);
    return x; 
}
//function that converts latitude to y
double convertLatitudetoY(double latitude){
    
    double y = kEarthRadiusInMeters * latitude * kDegreeToRadian;
    return y;  
}
//function that converts y to Latitude
double convertYtoLatitude(double y){
     
    double latitude = y / (kEarthRadiusInMeters * kDegreeToRadian);
    return latitude;  
} 
//function that converts x to longitude
double convertXtoLongitude(double x){
    
    double latAvg = kDegreeToRadian*((maxY + minY)/2.0); 
    double longitude = x/(kEarthRadiusInMeters * kDegreeToRadian * std::cos(latAvg));
    return longitude;  
}
//find closest poi index by position
int findClosestPOI(double x, double y){

    //initializes variables
    double minDistance =  std::numeric_limits<int>::max();
    double currentDistance=0;
    LatLon my_position(convertYtoLatitude(y), convertXtoLongitude(x));
    
    
    POIIdx closestPOI=0;
    int numPOI=getNumPointsOfInterest();
    //loop through all intersection and compare distance and name while storing the minimum
    for(int i = 0; i<numPOI; i++){
        currentDistance = findDistanceBetweenTwoPoints(std::make_pair(my_position, getPOIPosition(i))); 
        if(currentDistance < minDistance){ 
            minDistance = currentDistance;
            closestPOI = i; 
        }  
    }
    return closestPOI;    
}
/* 
 * Function to handle mouse press event
 * The current mouse position in the main canvas' world coordinate system is returned
 * A pointer to the application and the entire GDK event are also returned */
void act_on_mouse_click(ezgl::application *application, GdkEventButton *event, double x, double y)
{ 
  sharedIntersections.clear(); //clear shared intersections from find function
   
  if(search){
    showDirections=false; 
    navigationPath.clear(); 
  }
  search=false; 
  if(showNavigation){
    highlightIntersectionsNav(x,y);
    if(showDirections){       
        application->refresh_drawing(); //update map 
        popupDirections(application);
        showDirections=false;
    }
  }
  else{
    highlightIntersections(x,y); //highlight intersections
  }
  highlightPOI(x,y); //highlight POI
  application->refresh_drawing(); //update map 
}
//function sets highlight variables for Intersections
void highlightIntersections(double x, double y){
    //find closest intersection
  LatLon position = LatLon(convertYtoLatitude(y), convertXtoLongitude(x));
  int intersectionID = findClosestIntersection(position);
  int clickRadius = 10;

  //check if clicked on intersection or something else
  if((x<=intersections[intersectionID].x+clickRadius)&&(x>=intersections[intersectionID].x-clickRadius)
          &&(y<=intersections[intersectionID].y+clickRadius)&&(y>=intersections[intersectionID].y-clickRadius)){
   
    //unhighlight previously highlighted intersection
    if(intersections[intersections.size()-1].y!=-1){
        intersections[intersections[intersections.size()-1].y].highlight=false;
    }
    //unhighlight previously highlighted POI
    if(pointsOfInterest[pointsOfInterest.size()-1].y!=-1){
        pointsOfInterest[pointsOfInterest[pointsOfInterest.size()-1].y].highlight=false;
    }

    //setting bool highlight to true for closest intersection
    intersections[intersectionID].highlight=true;
    //storing highlighted info in reserved struct variable
    intersections[intersections.size()-1].y=intersectionID;
    intersections[intersections.size()-1].name=intersections[intersectionID].name; 
     
  } 
}
//function sets highlight variables for POI 
void highlightPOI(double x, double y){ 
    //find closest POI 
  int poiID = findClosestPOI(x,y); 
  int clickRadius = 7; 
  //check if clicked on POI or something else 
  if((x<=pointsOfInterest[poiID].x+clickRadius)&&(x>=pointsOfInterest[poiID].x-clickRadius) 
          &&(y<=pointsOfInterest[poiID].y+clickRadius)&&(y>=pointsOfInterest[poiID].y-clickRadius)){ 
   
    //unhighlight previously highlighted POI  
    if(pointsOfInterest[pointsOfInterest.size()-1].y!=-1){ 
        pointsOfInterest[pointsOfInterest[pointsOfInterest.size()-1].y].highlight=false;
    } 
    if(intersections[intersections.size()-1].y!=-1){ 
        intersections[intersections[intersections.size()-1].y].highlight=false; 
    } 

    //setting bool highlight to true for closest variable
    pointsOfInterest[poiID].highlight=true;
    //storing highlighted info in reserved struct variable
    pointsOfInterest[pointsOfInterest.size()-1].y=poiID;
    pointsOfInterest[pointsOfInterest.size()-1].name=pointsOfInterest[poiID].name;
 
  }
}
//set up buttons and widgets created with glade
void initial_setup(ezgl::application *application, bool new_window){
    
    GObject *searchbuttonObject = application -> get_object("SearchButton");
    g_signal_connect(searchbuttonObject, "clicked",  G_CALLBACK(search_button), application);
    
    GObject *textEntryObject = application -> get_object("textInput");
    g_signal_connect(textEntryObject, "activate", G_CALLBACK(text_entry), application);
    
    GObject *mapSelector = application -> get_object("MapSelect");
    g_signal_connect(mapSelector, "changed", G_CALLBACK(changeMap), application);
    
    //navEntry1
    GObject *textEntryObject1 = application -> get_object("navEntry1");
    g_signal_connect(textEntryObject1, "activate", G_CALLBACK(nav_entry_1), application);
       
    //navEntry2
    GObject *textEntryObject2 = application -> get_object("navEntry2");
    g_signal_connect(textEntryObject2, "activate", G_CALLBACK(nav_entry_2), application);
    
    GObject *okayHelp = application -> get_object("helpOkay");
    g_signal_connect(okayHelp, "clicked", G_CALLBACK(on_help_response), application);
    
    GObject *cancelHelp = application -> get_object("helpCancel");
    g_signal_connect(cancelHelp, "clicked", G_CALLBACK(on_help_response), application);
          
    application->create_button("Color Blind Mode", 6, colour_blind_mode);
    application->create_button("Night Mode", 7, night_mode);
    application->create_button("Display Subway Stations", 9, display_subway_stations);
    application->create_button("Navigate", 10, navigate_map); 
    application->create_button("Help", 10, display_help); 
}

//button to control the display of subway stations
void display_subway_stations(GtkWidget *widget, ezgl::application *application){
    
    displaySubwayStations = !displaySubwayStations;
    application->refresh_drawing();
}
//search button to located intersections
void search_button(GtkWidget *widget, ezgl::application *application){
    
    //text_entry(TextEntry, application);
    if(!showNavigation){
        GtkEntry* input = (GtkEntry*) application -> get_object("textInput");
        const char* text = gtk_entry_get_text(input);
        std::string str(text);
        findIntersections(str, application);
    }
    else{
        search=true;
        
        GtkEntry* input2 = (GtkEntry*) application -> get_object("navEntry2");
        const char* text2 = gtk_entry_get_text(input2);
        std::string str2(text2);
        IntersectionIdx endingIntersection = findIntersectionNavigate(text2,application);
        
        GtkEntry* input = (GtkEntry*) application -> get_object("navEntry1");
        const char* text = gtk_entry_get_text(input);
        std::string str(text);
        IntersectionIdx startIntersection = findIntersectionNavigate(text,application);
        navigationPath.clear();
        directions = "";
        
        if(startIntersection != -1 && endingIntersection != -1){
            navigationPath = findPathBetweenIntersections(startIntersection,endingIntersection,TIME_PENALTY);
                LatLon pos = getIntersectionPosition(startIntersection);
                start =startIntersection;
                end = endingIntersection;
                double x=convertLongitudetoX(pos.longitude());
                double y=convertLatitudetoY(pos.latitude());
                ezgl:: renderer *g = application->get_renderer();

                int zoomResizeAmount = 1000; //resize world to 1km of starting point
                ezgl::rectangle newZoom({x-zoomResizeAmount,y-zoomResizeAmount}, {x+zoomResizeAmount, y+zoomResizeAmount});
                g->set_visible_world(newZoom);

                application->refresh_drawing();//update map
        }
        else{
            directions.append("no directions found, please try again :)");
        }
        showDirections=true;
        application->refresh_drawing();
        popupDirections(application);
        sharedIntersectionsNav.clear();
    }
}
//search bar for user to enter in streets
void text_entry(GtkWidget *widget, ezgl::application *application){
    const gchar *input = gtk_entry_get_text((GtkEntry *) widget);
    std:: cout << input << std::endl;   
    std::string str(input);
    findIntersections(str, application);
    
}
//change the map to a different location 
void changeMap(GtkWidget *widget, ezgl:: application *application){
    
    //close current map and data base
    closeMap();
    
    //clear databases here
    freeDataStructures();    
    const gchar *map = gtk_combo_box_get_active_id((GtkComboBox*) widget);
    std::string mapName = map;
    std:: string newMapPath = "";
    
    //match the math path with selected drop down with the approtitate drop down
    if(mapName == "toronto"){
        newMapPath = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";
    }else if(mapName == "bejing"){
        newMapPath = "/cad2/ece297s/public/maps/beijing_china.streets.bin";
    }else if(mapName == "cairo-egypt"){
        newMapPath = "/cad2/ece297s/public/maps/cairo_egypt.streets.bin";
    }else if(mapName == "cap-town-south-africa"){
        newMapPath = "/cad2/ece297s/public/maps/cape-town_south-africa.streets.bin";
    }else if(mapName == "golden-horse-shoe"){
        newMapPath = "/cad2/ece297s/public/maps/golden-horseshoe_canada.streets.bin";
    }else if(mapName == "hamilton-canada"){
        newMapPath = "/cad2/ece297s/public/maps/hamilton_canada.streets.bin";
    }else if(mapName == "hong-kong"){
        newMapPath = "/cad2/ece297s/public/maps/hong-kong_china.streets.bin";
    }else if(mapName == "iceland"){
        newMapPath = "/cad2/ece297s/public/maps/iceland.streets.bin";
    }else if(mapName == "interlaken"){
        newMapPath = "/cad2/ece297s/public/maps/interlaken_switzerland.streets.bin";
    }else if(mapName == "london"){
        newMapPath = "/cad2/ece297s/public/maps/london_england.streets.bin";
    }else if(mapName == "moscow"){
        newMapPath = "/cad2/ece297s/public/maps/moscow_russia.streets.bin";
    }else if(mapName == "india"){
        newMapPath = "/cad2/ece297s/public/maps/new-delhi_india.streets.bin";
    }else if(mapName == "new-york"){
        newMapPath = "/cad2/ece297s/public/maps/new-york_usa.streets.bin";
    }else if(mapName == "rio"){
        newMapPath = "/cad2/ece297s/public/maps/rio-de-janeiro_brazil.streets.bin";
    }else if(mapName == "saint-helena"){
        newMapPath = "/cad2/ece297s/public/maps/saint-helena.streets.bin";
    }else if(mapName == "singapore"){
        newMapPath = "/cad2/ece297s/public/maps/singapore.streets.bin";
    }else if(mapName == "sydney"){
        newMapPath = "/cad2/ece297s/public/maps/sydney_australia.streets.bin";
    }else if(mapName == "tehran"){
        newMapPath = "/cad2/ece297s/public/maps/tehran_iran.streets.bin";
    }else if(mapName == "tokyo"){
        newMapPath = "/cad2/ece297s/public/maps/tokyo_japan.streets.bin";
    }

    //load the new map
    bool load_success = loadMap(newMapPath);
    if(!load_success) {
        std::cerr << "Failed to load map '" << newMapPath << "'\n";
        return;
    }
    
    //dtermine the osm path
    std:: string OSMDataPath = newMapPath.substr(0,newMapPath.size()-12);
    OSMDataPath += ".osm.bin";
    
    std:: cout<<"Loading OSM Data" + OSMDataPath<< std::endl;
    initalizeNewMap(application);    
    loadOSMData(OSMDataPath);
    
    application->refresh_drawing();
}
//intialize the new map with new coordinates 
void initalizeNewMap(ezgl:: application *application){
    
    //reset map
    std::unordered_map<StreetIdx,std:: vector<StreetSegmentIdx>>::iterator it;
    for (it=streetSegmentsOfStreet.begin(); it!=streetSegmentsOfStreet.end(); it++){
        it->second.clear();
        it->second.shrink_to_fit();   
    } 
    //reset starting min and max variables
    minX=std::numeric_limits<int>::max();
    minY=std::numeric_limits<int>::max();
    maxX=std::numeric_limits<int>::min();
    maxY=std::numeric_limits<int>::min();
   //finding bounds of city
    for (int i=0;i< getNumIntersections(); i++){
        
        //iterate and find maximum and minimum coordinates
        maxX = std::max(maxX, getIntersectionPosition(i).longitude());
        minX = std::min(minX, getIntersectionPosition(i).longitude());
        maxY = std::max(maxY, getIntersectionPosition(i).latitude());
        minY = std::min(minY, getIntersectionPosition(i).latitude());              
    }
    
    //storing intersection data in a vector //move to load map?
    //reserving an extra item to store id of previously highlighted intersections
    intersections.resize(getNumIntersections()+1);
    for (int k=0;k< getNumIntersections(); k++){
        
        intersections[k].name=getIntersectionName(k);
        intersections[k].y=convertLatitudetoY(getIntersectionPosition(k).latitude());
        intersections[k].x=convertLongitudetoX(getIntersectionPosition(k).longitude());     
    }
   
    //load the number of features    
    features.resize(getNumFeatures()+1);
    for (int i=0;i< getNumFeatures(); i++){
        
        features[i].name = getFeatureName(i);
        features[i].numPoints = getNumFeaturePoints(i);
        features[i].featureType = getFeatureType(i);
        features[i].highlight = false;
        features[i].prevHighlighted = false;
        features[i].points.resize(features[i].numPoints);
        for (int j = 0; j < features[i].numPoints; j++){
            features[i].points[j] = getFeaturePoint(i, j);
        }       
    }
    
    pointsOfInterest.resize(getNumPointsOfInterest()+1);
    for (int i=0;i< getNumPointsOfInterest(); i++){
        pointsOfInterest[i].type = getPOIType(i);
        pointsOfInterest[i].name = getPOIName(i);
        pointsOfInterest[i].y=convertLatitudetoY(getPOIPosition(i).latitude());
        pointsOfInterest[i].x=convertLongitudetoX(getPOIPosition(i).longitude());
        pointsOfInterest[i].osmid = getPOIOSMNodeID(i);
    }
    
    for(int i = 0; i < getNumStreetSegments(); i++){
        StreetSegmentInfo tempInfo = getStreetSegmentInfo(i);
        streetSegmentsOfStreet[tempInfo.streetID].push_back(i);
    }

    //initalize new world canvas
    ezgl::rectangle new_world({convertLongitudetoX(minX), convertLatitudetoY(minY)},
                {convertLongitudetoX(maxX), convertLatitudetoY(maxY)});
                      
    application->change_canvas_world_coordinates("MainCanvas",new_world);
}
//Loading OSM data into data structures
void loadOSMData(std:: string mapName){
     if(mapName != "/cad2/ece297s/public/maps/tokyo_japan.osm.bin"){
         
         //load OSM data
        loadOSMDatabaseBIN(mapName);

        
        for(int i = 0; i<getNumberOfWays(); i++){
            const OSMWay *currWay = getWayByIndex(i);
            OSMID id = currWay->id(); 

            //Check the tag of the currNode
            for (int j = 0; j < getTagCount(currWay); j++) {
                std::pair<std::string, std::string> tagPair = getTagPair(currWay, j);

                // Push nodes with the desired highway tags
                if (((tagPair.first == "highway")&&(tagPair.second == "motorway"))||((tagPair.first == "highway")&&(tagPair.second == "primary")) || ((tagPair.first == "highway")&&(tagPair.second == "trunk"))){
                     streetSegmentTypes[id]="highway";
                }else if((tagPair.first == "highway" && tagPair.second == "trunk")){
                    streetSegmentTypes[id]="long";
                }else if((tagPair.first == "highway" && tagPair.second == "tertiary")){
                    streetSegmentTypes[id]="medium";
                }else if((tagPair.first == "highway" && tagPair.second == "motorway_link") || (tagPair.first == "highway" && tagPair.second == "trunk_link")){
                    streetSegmentTypes[id]="highway-link";
                }
            }
        }

            // Loop through all OSM relations
            for (unsigned i = 0; i < getNumberOfRelations(); i++) {

                const OSMRelation *currRel = getRelationByIndex(i);

                // Check the tag of the currRel
                for (unsigned j = 0; j < getTagCount(currRel); j++) {

                    std::pair<std::string, std::string> tagPair = getTagPair(currRel, j);
                    // Push relations with the route=subway tag
                    if (tagPair.first == "route" && tagPair.second == "subway") {
                        subwayRoutes.push_back(currRel);
                        break;
                    }
                }
            }
                // For each subway line (relation), get its name, color, and members
                for (int i=0;i<subwayRoutes.size();i++) {

                    std::string color,name;
                    // Get subway line color and name
                    for (int j = 0; j < getTagCount(subwayRoutes[i]); j++) {
                        std::pair<std::string, std::string> tagPair = getTagPair(subwayRoutes[i], j);
                        if (tagPair.first == "colour") {
                            color=tagPair.second;
                        } 
                        else if (tagPair.first == "name") {	            
                            name=tagPair.second;            
                        }
                    }

                // Get relation members
                std::vector<TypedOSMID> route_members = getRelationMembers(subwayRoutes[i]);
                // Print subway station names	
                for(unsigned j = 0; j < route_members.size(); j++) {
                    // A member of type node represents a subway station
                    if(route_members[j].type() == TypedOSMID::Node) {
                        const OSMNode *currNode = nullptr;

                        // Node lookup by OSMID
                        for (unsigned k = 0; k < getNumberOfNodes(); k++) {
                             currNode = getNodeByIndex(k);
                             if (currNode->id() == route_members[j]){
                                break;
                            }
                        }
                        transitStations[currNode].color=color;
                        transitStations[currNode].name=name;

                        // Get the name tag of that node
                        for (unsigned k = 0; k < getTagCount(currNode); k++) {
                            std::pair<std::string, std::string> tagPair = getTagPair(currNode, k);
                            if (tagPair.first == "name") {                      
                                transitStations[currNode].stationName=tagPair.second;
                                transitStations[currNode].x=convertLongitudetoX(getNodeCoords(currNode).longitude());
                                transitStations[currNode].y=convertLatitudetoY(getNodeCoords(currNode).latitude());
                                break;
                            }
                        } 
                    }

           }
        }
        // Close layer 1 OSM database
        closeOSMDatabase();
    }
}
//toggle colorblind mode 
void colour_blind_mode(GtkWidget *widget, ezgl::application *application){
    colorBlindMode = !colorBlindMode; //toggle variable 
    if(colorBlindMode){ //chekc if on or off
        if(nightMode){  //turn night off in on
            nightMode = false;
        }
        setColorBlindColors();
    }else{
        setOriginalColors();
    }
    //refresh 
    application->refresh_drawing();   
}
//toggle night mode
void night_mode(GtkWidget *widget, ezgl::application *application){
    nightMode = !nightMode; //toggle mode 
    if(nightMode){ //check if on or off
        if(colorBlindMode){//turn off colorblind if on
            colorBlindMode = false;
        }
        setNightModeColors(); //set colors
    }else{
        setOriginalColors(); //set colors
    }
   // application->get_canvas().create_animation_renderer().set_color(backgroundNight) ;
    application->refresh_drawing();   
}
//set all colors to colorblind constants 
void setColorBlindColors(){
    unknownColor = ezgl::BLACK;
    parkColor = GreenSpaceColorBlind;
    breachColor = ezgl::YELLOW;
    lakeColor = LakeBlueColorBlind;
    riverColor = LakeBlueColorBlind;
    buildingColor =  ezgl::GREY_75;
    islandColor  = ezgl::KHAKI;
    greenspaceColor = GreenSpaceColorBlind;
    golfcourseColor = GreenSpaceColorBlind;
    streamColor  = StreamBlueColorBlind;
    backgroundColor  = backgroundColorBlind;
    streetColor  = streetColorBlind;
    borderColor  = borderColorBlind;
    POIColor = ezgl::BLUE;
    scaleColor=ezgl::BLACK;
    highwayColor=highwayColorBlind;
    majorStreetsColor=majorStreetsColorBlind;
    subwayNameColor=ezgl::BLACK;
    streetNamesColor=streetsNames;
    infoBoxColor=infoBoxColorBlind;
}
//set all colors to nighmode constants 
void setNightModeColors(){
    unknownColor = ezgl::BLACK;
    parkColor = GreenSpaceNight;
    breachColor = ezgl::YELLOW;
    lakeColor = LakeBlueNight;
    riverColor = LakeBlueNight;
    buildingColor =  ezgl::GREY_75;
    islandColor  = islandNight;
    greenspaceColor = GreenSpaceNight;
    golfcourseColor = GreenSpaceNight;
    streamColor  = StreamBlueNight;
    backgroundColor  = backgroundNight;
    streetColor  = streetNight;
    borderColor  = borderNight;
    POIColor = ezgl::BLUE;   
    highwayColor=highwayNight;
    majorStreetsColor=majorStreetsNight;
    scaleColor=ezgl::WHITE;
    subwayNameColor=ezgl::WHITE;
    streetNamesColor=ezgl::WHITE;
    infoBoxColor = infoBoxNight;
}
//set all colors to original values/constants
void setOriginalColors(){
    unknownColor = ezgl::BLACK;
    parkColor = GreenSpace;
    breachColor = ezgl::YELLOW;
    lakeColor = LakeBlue;
    riverColor = LakeBlue;
    buildingColor =  ezgl::GREY_75;
    islandColor  = ezgl::KHAKI;
    greenspaceColor = GreenSpace;
    golfcourseColor = GreenSpace;
    streamColor  = StreamBlue;
    backgroundColor  = background;
    streetColor  = street;
    borderColor  = border;
    POIColor = ezgl::BLUE;
    highwayColor=highway;
    majorStreetsColor=majorStreets;
    scaleColor=ezgl::BLACK;
    subwayNameColor=ezgl::BLACK;
    streetNamesColor=streetsNames;
    infoBoxColor=infoBox;
}

//display on the map 
void findIntersections(std::string streetNames, ezgl:: application *application){
    
    sharedIntersections.clear();
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
        return;
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
                sharedIntersections.push_back(sharedIntersectionsTemp[k]);
            }
        }
    }
    
    //if no intersections found return
    if(sharedIntersections.size() == 0){
        std:: cout << "no intersection found";
        return;
    }
    
    //bring the user to the location of the middle intersection and center it on the screen
    int middleIntersection = sharedIntersections.size()/2;
    LatLon pos = getIntersectionPosition(sharedIntersections[middleIntersection]);
    double x = convertLongitudetoX(pos.longitude());
    double y = convertLatitudetoY(pos.latitude());
    
    
    ezgl:: renderer *g = application->get_renderer();
    int zoomResizeAmount = 500; //resize world
    ezgl::rectangle newZoom({x-zoomResizeAmount,y-zoomResizeAmount}, {x+zoomResizeAmount, y+zoomResizeAmount});
    g->set_visible_world(newZoom);
    
    application->refresh_drawing();//update map
}
//function to draw scale
void drawScale(ezgl::renderer *g){
    
    ezgl::rectangle visibleWorld = g->get_visible_world(); //get coordinates of world in meters
    double worldWidth= visibleWorld.width();
    
    //draw scale on screen
    g->set_color(scaleColor);
    g->set_coordinate_system(ezgl::SCREEN);
    ezgl::rectangle visibleScreen = g->get_visible_screen();
    g->fill_rectangle({visibleScreen.right()-100,visibleScreen.top()-60}, {visibleScreen.right()-20, visibleScreen.top()-63});
    g->fill_rectangle({visibleScreen.right()-100,visibleScreen.top()-63}, {visibleScreen.right()-97, visibleScreen.top()-66});
    g->fill_rectangle({visibleScreen.right()-23,visibleScreen.top()-63}, {visibleScreen.right()-20, visibleScreen.top()-66});
    
    double pixelWidth=visibleScreen.width(); //get coordinates of screen in pixels
    
    int scale = (worldWidth/pixelWidth)*80; //multiply ratio scale factor by pixel width of scale drawn (80)
    std::string scaleText=std::to_string(scale) + " m";
   
    //print text
    g->set_text_rotation(0); 
    g->set_font_size(14);
    g->draw_text({visibleScreen.right()-60,visibleScreen.top()-75}, scaleText);
    
}
//function to draw subway stations
void drawSubwayStations(ezgl::renderer *g){
    
    //create map to keep track of what stations have been drawn
    ezgl::rectangle visibleWorld=g->get_visible_world();
    std::map<std::string, std::string> name;
    
    ezgl::surface *subWayIcon;
    
    //keep track of subway route/line color and draw station as such
    for (auto& i : transitStations) {
        std::transform(i.second.color.begin(), i.second.color.end(), i.second.color.begin(), ::toupper); 
        if(i.second.color=="RED"){
            subWayIcon = redSubway;
        }
        else if(i.second.color=="YELLOW"){ 
            subWayIcon = yellowSubway; 
        }
        else if(i.second.color=="GREEN"){
            subWayIcon = greenSubway;
        }
        else{ 
            subWayIcon = blueSubway;
        }
        g->set_coordinate_system(ezgl::WORLD);
        
        //drawing circles centered at station based on zoom factor 
        if(visibleWorld.area()<MINZOOMSMALLSTREET){
            if(name.count(i.second.stationName)!=1){ 
                g->draw_surface(subWayIcon,{i.second.x,i.second.y});
                name[i.second.stationName]=i.second.stationName; 
                g->set_text_rotation(0); 
                g->set_color(subwayNameColor);
                g->set_font_size(15);
                g->draw_text({i.second.x, i.second.y-30}, i.second.stationName); 
            }        
        }else if(visibleWorld.area()<MINZOOMMEDIUMSTREET){ 
            if(name.count(i.second.stationName)!=1){
                g->draw_surface(subWayIcon,{i.second.x,i.second.y}); 
                name[i.second.stationName]=i.second.stationName; 
                g->set_text_rotation(0); 
                g->set_color(subwayNameColor); 
                g->set_font_size(15); 
                g->draw_text({i.second.x, i.second.y-80}, i.second.stationName); 
            } 
        }else if(visibleWorld.area()<MINZOOMBIGSTREET){
            g->draw_surface(subWayIcon,{i.second.x,i.second.y});
        }
    }      
}
//function to free data structures
void freeDataStructures(){ 
    streetSegmentTypes.clear(); 
    intersections.clear(); 
    features.clear(); 
    pointsOfInterest.clear(); 
    transitStations.clear(); 
    subwayRoutes.clear();
    sharedIntersections.clear(); 
    navigationPath.clear();
    sharedIntersectionsNav.clear();
} 
