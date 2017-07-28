#include "circleMP.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <math.h>

circleMP::circleMP(nav_msgs::OccupancyGrid ob, int dr, int dx){
	o = ob;
	r = dr;
	x = dx;
	dTheta = x/(float)r;
	occupancyGrid.resize(ob.info.width, vector<int>(ob.info.height, 0));
	parseOGrid();
	geometry_msgs::Point zero;
	zero.x = 0;
	zero.y = 0;
	wfZero = worldToGF(zero);
}

circleMP::~circleMP(){}

geometry_msgs::Point circleMP::getNextPoint(){
	if(counter>counterMax){
		curR+=r;
		counter = 0;
		counterMax = (2*PI_F*curR/x);
	}
	curTheta = counter * dTheta;
	ideal.x = curR*cos(curTheta);
	ideal.y = curR*sin(curTheta);
	ideal = worldToGF(ideal);
	counter++;

	return findOpenSpot(ideal.x, ideal.y, wfZeroX.x wfZero.y);
}

void circleMP::parseOGrid()
{
    int oCounter = 0;
    int bufferSize = .75/og.info.resolution;
    for(int j = sh-1; j>=0; j--)
    {

        for(int i = 0; i<sw; i++)
        {

            occupancyGrid[i][j] = og.data[oCounter];
            if(occupancyGrid[i][j]){
                int startPosX = i-bufferSize;
                int startPosY = j-bufferSize;
                int endPosX   = i+bufferSize;
                int endPosY   = j+bufferSize;

                while (startPosX<0)startPosX++;
                while (startPosY<0)startPosY++;
                while (endPosX>=og.info.width)endPosX--;
                while (endPosY>=og.info.height)endPosY--;

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

geometry_msgs::Point findOpenSpot(float x1, float y1, float x2, float y2){

    geometry_msgs::Point p;
	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
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

    for(int x=(int)x1; x<maxX; x++)
    {
        if(steep)
        {
            if(!occupancyGrid[y][x])
            {
            	p.x = y;
            	p.y = x;
                return p;
            }
        }
        else
        {
            if(!occupancyGrid[x][y])
            {
            	p.x = x;
            	p.y = y;
                return p;
            }
        }

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    p.x = wfZeroX;
    p.y = wfZeroY;
    return gridToWF(p);
}

geometry_msgs::Point circleMP::worldToGF(geometry_msgs::Point p){
	geometry_msgs::Point msg;
	return msg;
}


geometry_msgs::Point circleMP::gridToWF(geometry_msgs::Point p){
	geometry_msgs::Point msg;
	return msg;
}