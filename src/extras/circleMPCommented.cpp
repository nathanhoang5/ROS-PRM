#include "circleMP.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <vector>

using namespace std;

circleMP::circleMP(nav_msgs::OccupancyGrid ob, int dr, int dx, int mR){
	o = ob;
    curR = dr;
	r = dr;
	x = dx;
    maxR = mR;
	dTheta = x/(float)r;
    counter = 0;
    counterMax = (2*PI_F*curR/x);
	occupancyGrid.resize(ob.info.width, vector<int>(ob.info.height, 0));
	parseOGrid();
	wfZero.x = 0;
	wfZero.y = 0;
	wfZero = worldToGF(wfZero);
}

circleMP::~circleMP(){}

geometry_msgs::Point circleMP::getNextPoint(){
    if(curR<maxR){  
        if(counter>counterMax){
            cout<<"New circle reached"<<endl;
            cout<<counter<<"  "<<counterMax<<endl;
            curR+=r;
            counter = 0;
            counterMax = (2*PI_F*curR/x);
        }
        curTheta = counter * dTheta;
        cout<<"curR: "<<curR<<endl;
        cout<<"curTheta: "<<curTheta<<endl;
        cout<<"sin(curTheta): "<<sin(curTheta)<<endl;
        ideal.x = curR*cos(curTheta);
        ideal.y = curR*sin(curTheta);
        cout<<"ideal.x: "<<ideal.x<<endl;
        cout<<"ideal.y: "<<ideal.y<<endl;
        ideal = worldToGF(ideal);
        float idealX = ideal.x;
        float idealY = ideal.y;
        cout<<"idealX: "<<idealX<<endl;
        cout<<"idealY: "<<idealY<<endl;
        float wfZeroX = wfZero.x;
        float wfZeroY = wfZero.y;
        counter++;

        cout<<"MADE IT HERE"<<endl;
    	return findOpenSpot(idealX, idealY, wfZeroX, wfZeroY);
    }
    else return wfZero;
}

void circleMP::parseOGrid()
{
    int oCounter = 0;
    int bufferSize = .75/o.info.resolution;
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
    cout<<"gridFrameP.x: "<<gridFrameP.x<<endl;
    gridFrameP.y = (p.y-(o.info.origin.position.y + o.info.height*o.info.resolution))*-1/o.info.resolution;
    cout<<"gridFrameP.y: "<<gridFrameP.y<<endl;
	return gridFrameP;
}


geometry_msgs::Point circleMP::gridToWF(geometry_msgs::Point p){
	geometry_msgs::Point worldFrameP;

	worldFrameP.x = (p.x*o.info.resolution)+o.info.origin.position.x;
    worldFrameP.y = (p.y*o.info.resolution/-1)+o.info.origin.position.y + o.info.height*o.info.resolution;

	return worldFrameP;
}



geometry_msgs::Point circleMP::findOpenSpot(float x1, float y1, float x2, float y2){

    cout<<"x1: "<<x1<<endl;
    cout<<"y1: "<<y1<<endl;
    cout<<"x2: "<<x2<<endl;
    cout<<"y2: "<<y2<<endl;
    geometry_msgs::Point p;
    vector<geometry_msgs::Point> pointList;

	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    bool waitForEnd = (!steep&&x1>x2)||(steep&&y1>y2);
    cout<<"Wait for end?: "<<waitForEnd<<endl;
    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
    {

        std::swap(x1, x2);
        std::swap(y1, y2);
        cout<<"Swapped, x2 = "<<x2;
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;
    cout<<"maxX: "<<maxX<<endl;
    int prevX, prevY;

    for(int x=(int)x1; x<maxX; x++)
    {
        cout<<"x: "<<x<<endl;
        cout<<"y: "<<y<<endl;
        cout<<"------"<<endl;
        if(steep)
        {
            if(!occupancyGrid[y][x]&&!waitForEnd)
            {
            	p.x = y;
            	p.y = x;
                cout<<"p.x: "<<p.x<<endl;
                cout<<"p.y: "<<p.y<<endl;
                return p;
            }
            else if(!occupancyGrid[y][x]&&waitForEnd){
                p.x = y;
                p.y = x;
                pointList.push_back(p);
            }
        }
        else
        {
            if(!occupancyGrid[x][y]&&!waitForEnd)
            {
                if(x==130&&y==173)cout<<"Evaluated"<<endl;
            	p.x = x;
            	p.y = y;
                cout<<"p.x: "<<p.x<<endl;
                cout<<"p.y: "<<p.y<<endl;
                return p;
            }
            else if(!occupancyGrid[x][y]&&waitForEnd){
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
        cout<<"p.x: "<<pointList.back().x<<endl;
        cout<<"p.y: "<<pointList.back().y<<endl;
        return pointList.back();
    }


    p.x = wfZero.x;
    p.y = wfZero.y;
    cout<<"p.x: "<<p.x<<endl;
    cout<<"p.y: "<<p.y<<endl;
    return gridToWF(p);
}