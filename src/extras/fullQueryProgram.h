#pragma once
#include <SDL.h>
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>

enum class GameState{PLAY,EXIT};

class pQuery
{
public:
	pQuery();
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
	void gameLoop();
	void processInput();

	void createObstacle();
	void populate();
	bool notObstructed( int x1, int y1, int x2, int y2);
	bool Line(  float x1, float y1,  float x2,  float y2);
	void connect();
	int maxNum(int a, int b);
	int minNum(int a, int b);
	void redrawSF();
	void printCn();
	void fillROSNodeArray();
	void query();
	int getMoveDist(int a, int b);
	void foundNode(int a);
	bool notFound(int a);
	void addNodes(int a);
	void clearQueueList();
	void redrawFin();
	void clearRenderer();
	void populateTestMap();

	SDL_Window* _window;
	SDL_Renderer* _renderer;
	int _screenWidth;
	int _screenHeight;
	GameState _gameState;

};

