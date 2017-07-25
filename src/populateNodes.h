#pragma once
#include <SDL.h>
#include <prm/node.h>
#include <prm/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>

enum class GameState{PLAY,EXIT};

class populateNodes
{
public:
	populateNodes(int nN, nav_msgs::OccupancyGrid ob);
	~populateNodes();
    int nNodes;
	void run(float sx, float sy, int ex, int ey, int mD);
    prm::nodeArray n;




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
