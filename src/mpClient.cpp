#include "ros/ros.h"
#include "prm/circularMissionPlan.h"
#include <SDL.h>
#include <cstdlib>

using namespace std;

geometry_msgs::Point worldToGF(geometry_msgs::Point p, nav_msgs::OccupancyGrid o){
  geometry_msgs::Point gridFrameP;

    gridFrameP.x = (p.x-o.info.origin.position.x)/o.info.resolution;
    gridFrameP.y = (p.y-(o.info.origin.position.y + o.info.height*o.info.resolution))*-1/o.info.resolution;

  return gridFrameP;
}

void mapCallback(const nav_msgs::OccupancyGrid& o)
{

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<prm::circularMissionPlan>("circleMP");
  prm::circularMissionPlan srv;
  // nav_msgs::OccupancyGrid o;

  // //Create occupancy grid
  // o.info.width = 300;
  // o.info.height = 300;
  // o.info.origin.position.x = -15;
  // o.info.origin.position.y = -15;
  // o.info.resolution = .1;
  // for(int i = 0; i<o.info.width*o.info.height; i++){
  //   o.data.push_back(0);
  // }

  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Window* _window = SDL_CreateWindow("Mission Planner", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, o.info.width, o.info.height, SDL_WINDOW_SHOWN);
  SDL_Renderer* _renderer = SDL_CreateRenderer(_window, -1, SDL_RENDERER_ACCELERATED);
  if (_window == nullptr)
  {
        cout<<"Window no create"<<endl;
   }
  SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
  SDL_RenderClear(_renderer);
  vector<vector<int>> occupancyGrid;
  occupancyGrid.resize(o.info.width, vector<int>(o.info.height, 0));
  int oCounter = 0;
  SDL_SetRenderDrawColor(_renderer, 75, 0, 130, 255);  
    for(int j = o.info.height-1; j>=0; j--)
    {

        for(int i = 0; i<o.info.width; i++)
        {

            occupancyGrid[i][j] = o.data[oCounter];
            if(occupancyGrid[i][j])SDL_RenderDrawPoint(_renderer,i,j);
            oCounter++;
        }
    }

  //Fill srv.request
  srv.request.o = o;
  srv.request.dr = 3;
  srv.request.dx = 3;
  srv.request.maxRadius = 50;

  //max survey points
  int maxPoints = 80, nodeSize = 3;

   for(int i = 0; i<maxPoints; i++){
    if (client.call(srv))
    {
      //Display to screen
      geometry_msgs::Point p = worldToGF(srv.response.p, o);
      // // cout<<"Iteration: "<<i<<endl;
      // cout<<"("<<p.x<<",";
      // cout<<p.y<<"),";
      // // cout<<"-------------"<<endl;
      int startX = p.x;
      int startY = p.y;
      SDL_Rect startRect;
      startRect.h = nodeSize;
      startRect.w = nodeSize;
      startRect.x = startX-nodeSize/2;
      startRect.y = startY-nodeSize/2;
      SDL_SetRenderDrawColor(_renderer, 0, 139, 139, 255);
      SDL_RenderFillRect(_renderer, &startRect);

      // ros::Duration(0.5).sleep();
    }

    else
    {
      ROS_ERROR("Failed to call mission planner");
      
    }

   }

  SDL_RenderPresent(_renderer);
  SDL_RenderPresent(_renderer);


  bool _gameState = true;
  while(_gameState){
    SDL_Event evnt;
        while (SDL_PollEvent(&evnt) == true)
        {
            switch (evnt.type)
            {
            //If exit is clicked, close application
              case SDL_QUIT:
                  _gameState = false;
                  break;
            }
        }
  }


  SDL_DestroyWindow(_window);
  SDL_Quit();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MP_TEST_CLIENT");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("projected_map", 1, mapCallback);

    ros::spin();
    return 0;
}
