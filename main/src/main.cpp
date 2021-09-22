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
#include <string>

#include "m1.h"
#include "m2.h"
#include "m3_internal.h"
#include "m4_internal.h"

#include <chrono>
#define TIME_LIMIT 50

//Program exit codes
constexpr int SUCCESS_EXIT_CODE = 0;        //Everyting went OK
constexpr int ERROR_EXIT_CODE = 1;          //An error occured
constexpr int BAD_ARGUMENTS_EXIT_CODE = 2;  //Invalid command-line usage

//The default map to load if none is specified
std::string default_map_path = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";


// The start routine of your program (main) when you are running your standalone
// mapper program. This main routine is *never called* when you are running 
// ece297exercise (the unit tests) -- those tests have their own main routine
// and directly call your functions in /libstreetmap/src/ to test them.
// Don't write any code in this file that you want run by ece297exerise -- it 
// will not be called!
int main(int argc, char** argv) {
    
    std::vector<DeliveryInf> deliveries;
    std::vector<IntersectionIdx> depots;


    
    std::string map_path;
    if(argc == 1) {
        //Use a default map
        map_path = default_map_path;
    } else if (argc == 2) {
        //Get the map from the command line
        map_path = argv[1];
    } else {
        //Invalid arguments
        std::cerr << "Usage: " << argv[0] << " [map_file_path]\n";
        std::cerr << "  If no map_file_path is provided a default map is loaded.\n";
        return BAD_ARGUMENTS_EXIT_CODE;
    }

    //Load the map and related data structures
    bool load_success = loadMap(map_path);
    if(!load_success) {
        std::cerr << "Failed to load map '" << map_path << "'\n";
        return ERROR_EXIT_CODE;
    }

    std::cout << "Successfully loaded map '" << map_path << "'\n";
    
 
    drawMap();
    //Clean-up the map data and related data structures
    
    deliveries = {DeliveryInf(134204, 32615), DeliveryInf(113281, 117913), DeliveryInf(74966, 45024), DeliveryInf(135221, 51749), DeliveryInf(40756, 38216), DeliveryInf(102380, 48326), DeliveryInf(86334, 101772), DeliveryInf(34975, 81132), DeliveryInf(82672, 87076), DeliveryInf(42199, 87321), DeliveryInf(27101, 82634), DeliveryInf(119658, 56035), DeliveryInf(29435, 28262), DeliveryInf(136515, 72388), DeliveryInf(65038, 96006), DeliveryInf(149474, 140389), DeliveryInf(81186, 125686), DeliveryInf(105969, 37007), DeliveryInf(103341, 133821), DeliveryInf(137127, 122694), DeliveryInf(117691, 82535), DeliveryInf(95563, 121164), DeliveryInf(145491, 10229), DeliveryInf(121985, 42614), DeliveryInf(106896, 109313)};
    depots = {10, 38701};
    double turn_penalty = 15.000000000;
    travelingCourier(deliveries, depots, turn_penalty);


            
  
   // std:: cout<<"actual time is " << actual;
    std::cout << "Closing map\n";
    closeMap(); 


    return SUCCESS_EXIT_CODE;
}
