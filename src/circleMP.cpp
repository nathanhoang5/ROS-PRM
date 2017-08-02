#include "ros/ros.h"
#include "circleMP.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <vector>

using namespace std;

circleMP::circleMP(int dr, int dx, int mR){
	
    curR = dr;
	r = dr;
	x = dx;
    maxR = mR;
	dTheta = x/(float)r;
    counter = 0;
    counterMax = (2*PI_F*curR/x);
	
}

circleMP::~circleMP(){}

geometry_msgs::Point circleMP::getNextPoint(nav_msgs::OccupancyGrid ob){
    o = ob;
    occupancyGrid.clear();
    occupancyGrid.resize(ob.info.width, vector<int>(ob.info.height, 0));
    parseOGrid();
    wfZero.x = 0;
    wfZero.y = 0;
    wfZero = worldToGF(wfZero);
    cout<<"Radius: "<<r<<endl;
    cout<<"Counter val: "<<counter<<endl;
    cout<<"Counter max: "<<counterMax<<endl;
    if(curR<maxR){  
        if(counter>counterMax){
            cout<<counter<<"  "<<counterMax<<endl;
            curR+=r;
            dTheta = x/(float)curR;
            counter = 0;
            counterMax = (2*PI_F*curR/x);
        }
        curTheta = counter * dTheta;
        ideal.x = curR*cos(curTheta);
        ideal.y = curR*sin(curTheta);
        cout<<"idealX WF: "<<ideal.x<<endl;
        cout<<"idealY WF: "<<ideal.y<<endl;
        ideal = worldToGF(ideal);
        float idealX = ideal.x;
        float idealY = ideal.y;
        cout<<"idealX: "<<idealX<<endl;
        cout<<"idealY: "<<idealY<<endl;
        float wfZeroX = wfZero.x;
        float wfZeroY = wfZero.y;
        counter++;

    	return findOpenSpot(idealX, idealY, wfZeroX, wfZeroY);
    }
    else {
        wfZero.x = 0;
        wfZero.y = 0;
        ROS_INFO("MAX RADIUS REACHED");
        return wfZero;
    }
}

void circleMP::parseOGrid()
{
    cout<<"Grid ID: "<<o.info.map_load_time;
    int oCounter = 0;
    int bufferSize = 1/o.info.resolution;
    for(int j = o.info.height-1; j>=0; j--)
    {

        for(int i = 0; i<o.info.width; i++)
        {

            occupancyGrid[i][j] = o.data[oCounter];
            if(occupancyGrid[i][j]){
                int startPosX = i-bufferSize;
                int startPosY = j-bufferSize;
                int endPosX   = i+bufferSize;
                int endPosY   = j+bufferSize;

                while (startPosX<0)startPosX++;
                while (startPosY<0)startPosY++;
                while (endPosX>=o.info.width)endPosX--;
                while (endPosY>=o.info.height)endPosY--;

                for (int rowNum=startPosX; rowNum<=endPosX; rowNum++) {
                    for (int colNum=startPosY; colNum<=endPosY; colNum++) {
                        occupancyGrid[rowNum][colNum]=1;
                    }
                }

            }
            oCounter++;
        }
    }
}

geometry_msgs::Point circleMP::worldToGF(geometry_msgs::Point p){
	geometry_msgs::Point gridFrameP;

    gridFrameP.x = (p.x-o.info.origin.position.x)/o.info.resolution;
    gridFrameP.y = (p.y-(o.info.origin.position.y + o.info.height*o.info.resolution))*-1/o.info.resolution;
	return gridFrameP;
}


geometry_msgs::Point circleMP::gridToWF(geometry_msgs::Point p){
	geometry_msgs::Point worldFrameP;

	worldFrameP.x = (p.x*o.info.resolution)+o.info.origin.position.x;
    worldFrameP.y = (p.y*o.info.resolution/-1)+o.info.origin.position.y + o.info.height*o.info.resolution;
	return worldFrameP;
}



geometry_msgs::Point circleMP::findOpenSpot(float x1, float y1, float x2, float y2){

    geometry_msgs::Point p;
    vector<geometry_msgs::Point> pointList;

	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    bool waitForEnd = (!steep&&x1>x2)||(steep&&y1>y2);
    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
    {

        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;
    int prevX, prevY;

    for(int x=(int)x1; x<maxX; x++)
    {

        if(steep)
        {
            if(!occupancyGrid[y][x]&&!waitForEnd&&inGrid(y,x))
            {
            	p.x = y;
            	p.y = x;
                cout<<"NO WAIT, STEEP p.x: "<<p.x<<endl;
                cout<<"p.y: "<<p.y<<endl;
                cout<<"OG AT (x,y): "<<occupancyGrid[p.x][p.y]<<endl;
                return p;
            }
            else if(!occupancyGrid[y][x]&&waitForEnd&&inGrid(y,x)){
                p.x = y;
                p.y = x;
                pointList.push_back(p);
            }
        }
        else
        {
            if(!occupancyGrid[x][y]&&!waitForEnd&&inGrid(x,y))
            {
            	p.x = x;
            	p.y = y;
                cout<<"NO WAIT, STEEP p.x: "<<p.x<<endl;
                cout<<"p.y: "<<p.y<<endl;
                cout<<"OG AT (x,y): "<<occupancyGrid[p.x][p.y]<<endl;
                return p;
            }
            else if(!occupancyGrid[x][y]&&waitForEnd&&inGrid(x,y)){
                p.x = x;
                p.y = y;
                pointList.push_back(p);
            }

        }
        prevX = x;
        prevY = y;
        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    if(pointList.size()>0){
        cout<<"WAIT p.x: "<<pointList.back().x<<endl;
        cout<<"p.y: "<<pointList.back().y<<endl;
        cout<<"OG AT (x,y): "<<occupancyGrid[p.x][p.y]<<endl;
        return pointList.back();
    }


    p.x = wfZero.x;
    p.y = wfZero.y;
    cout<<"RETURN WF ZERO p.x: "<<p.x<<endl;
    cout<<"RETURN WF ZERO p.y: "<<p.y<<endl;
    return p;
}

bool circleMP::inGrid(int x, int y){
    if(x>0&&x<o.info.width&&y>0&&y<o.info.height)
        return true;
    else
        return false;
}