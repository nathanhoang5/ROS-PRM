#pragma once
#include <SDL.h>
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>

enum class GameState{PLAY,EXIT};

class MainGame
{
public:
	MainGame(int nN, nav_msgs::OccupancyGrid ob);
	~MainGame();
    int nNodes;
	void run(float sx, float sy, int ex, int ey, int mD);
    beginner_tutorials::nodeArray n;




private:
	void initSystems();
	void gameLoop();
	void processInput();

	void createObstacle();
	void populate();
	bool Line(  float x1, float y1,  float x2,  float y2);
	void connect();
	void redrawSF();
	void fillROSNodeArray();
	void parseOGrid();
	void redrawFin();


	SDL_Window* _window;
	SDL_Renderer* _renderer;
	int _screenWidth;
	int _screenHeight;
	GameState _gameState;

};
