/**
 * @file main.cpp
 *
 * @brief:
 * 	CppND-Route-Planning-Project: Main function for Route Planning Project. 
 *      The main.cpp controls the flow of the program, accomplishing four primary tasks:
 *   	- The OSM data is read into the program.
 *  	- A RouteModel object is created to store the OSM data in usable data structures.
 *   	- A RoutePlanner object is created using the RouteModel. This planner will eventually carry out the A* search on the model data and store the search results in the RouteModel.
 *   	- The RouteModel data is rendered using the IO2D library. Have a look at the video below for a brief overview of this file:
 *
 * @ingroup:
 * 	CppND-Route-Planning-Project
 *
 * @author:
 * 	Eva Liu - evaliu2046@gmail.com
 * 
 * @date:
 * 	2019/Jun/02
 *
 */
#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i]; 
    }
    else {
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;    
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    float start_x, start_y, end_x, end_y;
    // user input for these values using std::cin. Pass the user input to the
    std::cout << "The cordinate will start from the bottom left corner (0,0), and end with the top right corner (100, 100)." << "\n";
    std::cin >> start_x;
    std::cin >> start_y;
    std::cin >> end_x;
    std::cin >> end_y;
    // RoutePlanner object below.

    // Build Model.
    RouteModel model{osm_data};

    // Perform search and render results.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();
    std::cout <<"The Distance is: "<< route_planner.GetDistance() <<"\n";
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}

