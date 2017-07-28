#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

class circleMP
{
public:
	circleMP(nav_msgs::OccupancyGrid ob, int dr, int dx);
	~circleMP();
	geometry_msgs::Point getNextPoint();

private:
	nav_msgs::OccupancyGrid o;
	vector<vector<int>> occupancyGrid;
	const float  PI_F=3.14159265358979f;
	int r, x, counterMax, counter, currentRadius;
	geometry_msgs::Point wfZero;
	geometry_msgs::Point ideal;
	float curR, curTheta, dTheta;


	void parseOGrid();
	geometry_msgs::Point findOpenSpot(float x1, float y1, float x2, float y2);
	geometry_msgs::Point worldToGF(geometry_msgs::Point p);
	geometry_msgs::Point gridToWF(geometry_msgs::Point p);
};
