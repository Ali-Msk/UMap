
![Banner](/images/logo.jpg)
# UMap
<!--![Language Stats](/images/languageStats.png)<br/>-->
UMap is a tourist-friendly map application that shows important information for different cities around the world. Written using C++, retrieves data using the OpenStreetMap Database API, and draws graphics using GTK.<br/>

NOTE: This project was made for the course ECE297 at the University of Toronto, and therefore, the source code is not available to the public.

## Main Features
* Use buttons to move around and zoom in/out
* Dropdown menu to select maps for different cities
* Display streets, street names, parks, rivers, lakes, buildings, points of interests, etc...
* Display subway stations
* Click POI's or intersections to view more information
* Search bar to search up intersections
* Find optimal path between two intersections
* Display easy-to-follow driving instructions to get from one point to another
* Dark Mode
* Colour Blind Support
* Help Box

## Algorithms
### Setting up a Graph data structure
Our map represents the information retrieved from the OSM Database as a directed, weighted graph. The nodes of the graph represent city intersections, and the edges represent street segments that connect two intersections. The weight of each edge represents the travel time to get from one intersection to another, which was calculated from the length of the street divided by the speed limit. Within the program, data is stored within structs, vectors, unordered maps, and sets.

### Pathfinding using A* Search Algorithm
Our map implements the A* search algorithm to search for the shortest path between two intersections. Based on travel time between intersections, we used a greedy approach to always travel to the next intersection with the quickest travel time. On top of that, our algorithm takes into account a heuristic calculated with the euclidean distance between current intersection to destination, divided by the maximum speed limit in the city, this heuristic guides the algorithms towards to destination allowing us to arrive more quickly.
### Traveling Salesman/Courier Problem - NP Hard Problem
Given a set of dropoff/pickup points, and a set of start/end intersections, our algorithm attempts to find the optimal path that reaches all the intersections in the appropriate order. Since this type of problem is an example of an NP-Hard problem, and no optimal solutions exist our algorithm finds a base solution then continues to make improvements until we end up with a relatively optimal path within a 50 second time limit.

For our initial base solution, we try starting at every possible depot(start point) and use a greedy algorithm to always travel to the next closest pickup or dropoff point from the current location. Then, using the best base solution, we try swapping the order of four random intersections continuously to look for a more optimal path for a maximum time of 50 seconds.

## Screenshots
| Main Screen  | Zoom-Levels |
| ------------- | ------------- |
| ![Main Screen](/images/map.png)  | ![Zoom Levels](/images/zoom_levels.gif) |

| Dark Mode  | Colour Blind Mode |
| ------------- | ------------- |
| ![Dark Mode](/images/nightmode.png)  | ![Colour Blind Mode](/images/colorMap.png) |

| Path-Finding  | Search |
| ------------- | ------------- |
| ![Path Finding](/images/Navigation.png)  | ![Search](/images/search.gif) |

| Subways  | Points of Interest |
| ------------- | ------------- |
| ![Subways](/images/subwayDisplay.png)  | ![Points of Interest](/images/POI.gif) |
