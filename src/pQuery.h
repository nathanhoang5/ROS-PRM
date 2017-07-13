#pragma once
#include <SDL.h>
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>

enum class GameState{PLAY,EXIT};

class pQuery
{
public:
	pQuery(int nN);
	~pQuery();
    int tester;
    int sx;
    int sy;
    int ex;
    int ey;
	void run();
    beginner_tutorials::nodeArray n;



private:
	void initSystems();
	void fillLocalNodeArray();
	void setStartEnd();
	void resetConnection(int nodeNumber);
	void gameLoop();
	void processInput();
	void createObstacle();
	void redrawSF();
	void fillROSNodeArray();
	void connect(int a);
	bool Line(  float x1, float y1,  float x2,  float y2);
	void query();
	int getMoveDist(int a, int b);
	void foundNode(int a);
	bool notFound(int a);
	void addNodes(int a);
	void clearQueueList();
	void redrawFin();
	void clearRenderer();
	void populateTestMap();
	void drawNodes();

	SDL_Window* _window;
	SDL_Renderer* _renderer;
	int _screenWidth;
	int _screenHeight;
	GameState _gameState;

};
