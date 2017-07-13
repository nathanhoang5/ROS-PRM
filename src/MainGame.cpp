#include "MainGame.h"
#include <iostream>
#include <queue>
#include <SDL.h>
#include <chrono>
#include <ratio>
#include <ctime>
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>

using namespace std;


//Number of nodes

int numNodes;
//Start and end positions
int startX;
int startY;
int endX;
int endY;

//Screen height and screen width
const int sw = 500;
const int sh = 300;

//Binary Occupancy map
int occupancyGrid[sw][sh];

//Size of node rectangles
const int nodeSize = 3;

//True after start and end have been selected, takes spacebar input
bool stillRunning = true;

//True at start, allows selection of start and end points
bool selectPts = false;

//Number of obstacles
const int numObs = 3;

//Array storing all obstacles
SDL_Rect* obs = new SDL_Rect[numObs];

//Max distance allowed between nodes
int maxNodeDist = 5000;

//Stores path from start to finish
string pathList;

//List of closed nodes
int * closedNodeList;

//List of new connections to be added, cleared for each new node evaluated
int * addedNodes;

//Counter for index of closed node array
int closedCounter = 0;

//Counter for index of added nodes array
int aNCounter = 0;

double runTime = 0;




//Each point stored as a node
class node
{
	//Current position
	int xPos;
	int yPos;
	//Total distance already travelled to reach the node
	int level = 0;
	//Priority=distance from start node
	int priority;  // smaller: higher priority
	//Parent node
	int parent;
	//Value of node in node array
	int arrayValue;
	//List of node connections
	int * connections = new int [numNodes];
	//Counter for index of added nodes array
	int connectionCounter = 0;



public:
	node(int xp, int yp, int d, int p, int a)
	{
		xPos = xp; yPos = yp; level = d; priority = p; arrayValue = a;
	}

	int getxPos() const { return xPos; }
	int getyPos() const { return yPos; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }
	int getParent() const { return parent; }
	int getArrayValue() const { return arrayValue; }
	int getConnection(int n) {	return connections[n];	}
	int getConnectionCounter(){  return connectionCounter;  }
	int* getConnectionArray(){  return connections; }


	//Sets movement cost to get to this node, then the priority (for the priority queue)
	void setPriority(int pD)
	{
		level = pD;
		priority = level+estimate();
	}

	// Estimation function for the remaining distance to the goal (can be used in priority for A*)
	const int & estimate() const
	{
		static int xd, yd, d;
		xd = endX - xPos;
		yd = endY - yPos;
		d = static_cast<int>(sqrt(xd*xd + yd*yd));
		return(d);
	}

	//Sets the parent of node
	void setParent(int par) {
		parent = par;
	}

	//Initializes all values of connection array
	void initCArray() {
		for (int i = 0; i < numNodes; i++) {
			connections[i] = -1;
		}
	}

	//Adds connection to a neighboring node
	void addConnection(int c) {
		connections[connectionCounter] = c;
		connectionCounter = connectionCounter + 1;
	}

	//Prints all connections
	void printConnections() {
		for (int i = 0; i < numNodes; i++) {
			if (connections[i] == -1) break;
			else {
				cout << connections[i];
			}
		}
	}

};

//Used to determine order of priority queue (lower priority/move distance is higher)
class CompareNode {
public:
	bool operator()(node & n1, node & n2)
	{
		return n1.getPriority() > n2.getPriority();
	}
};


//List of nodes
node** nodeList;

//Initialize window parameters
MainGame::MainGame(int nN)
{
    numNodes = nN;
	_window = nullptr;
	_renderer = nullptr;
	_screenWidth = sw;
	_screenHeight = sh;
	_gameState = GameState::PLAY;

	nodeList = new node* [numNodes];
	closedNodeList = new int[numNodes];
	addedNodes = new int[numNodes];

}
//Can be called to exit application when error is thrown
void fatalError(string errorString) {
	cout << errorString << endl;
	cout << "Enter any key to quit...";
	int tmp;
	cin >> tmp;
	SDL_Quit();
	exit(1);
}

//Destructor?? I don't know what this is...
MainGame::~MainGame()
{
    //delete []nodeList;
    //delete []closedNodeList;
    //delete []addedNodes;
}


//Called from main class, starts application depending on if selectPoints==true, then starts gameLoop()
void MainGame::run(int sx, int sy, int ex, int ey, int mD) {

	//maxDist
	initSystems();

    createObstacle();
    createObstacle();
    SDL_RenderPresent(_renderer);
    SDL_RenderPresent(_renderer);



    cout << "Press space to populate" << endl;

    startX = sx;
    startY = sy;
    endX = ex;
    endY = ey;
    maxNodeDist = mD;

    redrawSF();
    redrawSF();
	gameLoop();

	SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
	SDL_RenderClear(_renderer);
    SDL_DestroyWindow(_window);
    SDL_Quit();

    for(int i = 0; i<numNodes; i++) delete nodeList[i];
    delete []nodeList;
    delete []closedNodeList;
    delete []addedNodes;
	//cout<<"GameState = EXIT"<<endl;
}

//Create window and initialize lists, makes white background
void MainGame::initSystems() {

	SDL_Init(SDL_INIT_EVERYTHING);
	_window = SDL_CreateWindow("Probabilistic Roadmap", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, _screenWidth, _screenHeight, SDL_WINDOW_SHOWN);
	_renderer = SDL_CreateRenderer(_window, -1, SDL_RENDERER_ACCELERATED);
	if (_window == nullptr) {
		fatalError("SDL Window could not be created!");
	}
	SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
	SDL_RenderClear(_renderer);

	for (int i = 0; i < numNodes; i++) {
		closedNodeList[i] = -1;
		addedNodes[i] = -1;
	}
	closedNodeList[0] = 0;
	closedCounter = closedCounter + 1;
    stillRunning = true;
	selectPts = false;



}

//If there is no error, continue to process input
void MainGame::gameLoop() {
	while (_gameState != GameState::EXIT) {
		processInput();
		//drawGame();
	}

}

//Keeps track of stage (populate, connect, query)
int counter = 0;
//Records time to execute algorithm
clock_t start, i1, i2, i3, i4, endClock;
clock_t t1, t2;
//True if selecting start position, false if selecting end position
bool selectStart = true;
//Takes user input
void MainGame::processInput() {
	SDL_Event evnt;
	while (SDL_PollEvent(&evnt) == true) {
		switch (evnt.type) {
            //If exit is clicked, close application
            case SDL_QUIT:
				_gameState = GameState::EXIT;
				break;
			//Mouse is clicked
			case SDL_MOUSEBUTTONDOWN:
				//Select start position
				if (selectPts&&selectStart) {
					SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
					SDL_RenderClear(_renderer);
					//createObstacle();
					startX = evnt.button.x;
					startY = evnt.button.y;
					selectStart = false;
					cout << "Select end point" << endl;
				}
				//Select end position
				else if (selectPts&&selectStart == false) {
					endX = evnt.button.x;
					endY = evnt.button.y;
					selectStart = true;
					selectPts = false;
					stillRunning = true;
					createObstacle();
					createObstacle();
					redrawSF();
					cout << "Press space to populate map" << endl;
				}
			//case SDL_MOUSEMOTION:
			//	cout << evnt.motion.x << " " << evnt.motion.y << endl;

			//TODO: Fix clock
			//If spacebar pressed
			case SDL_KEYDOWN:
				if (stillRunning) {
					if (evnt.key.keysym.scancode == SDL_SCANCODE_SPACE) {
						if (counter == 0) {
							start = clock();
							populate();
							//populateTestMap();
							createObstacle();
							createObstacle();

							i1 = clock();
							cout << "Populate time: "<< (i1-start)/(double) CLOCKS_PER_SEC*1000 <<endl;

							cout << "Press space to connect nodes" << endl;
							counter = counter + 1;
						}
						else if (counter == 1) {
							i2 = clock();
							connect();
                            fillROSNodeArray();
							i3 = clock();
							cout << "Connect time: "<< (i3-i2)/(double) CLOCKS_PER_SEC*1000 <<endl;
							//cout << "Press space to find path (A*)" << endl;
                            cout << "----------------------"<<endl;
							counter = 0;
                            /*for(int i = 0;i<numNodes;i++){
                                cout<<"Node "<<nodeList[i]->getArrayValue()<<":  x="<<nodeList[i]->getxPos()<<", y="<<nodeList[i]->getyPos()<<endl;
                            }*/
							stillRunning = false;
							//cout << "Collision check time: " << timeTaken << endl;
						}

					}
				}
		}
	}
}


//Populates map of numNodes nodes
void MainGame::populate() {

	//Used to populate array
	static node* a;

	//Starting point
	nodeList[0] = new node(startX, startY, 0, 10000, 0);
	nodeList[0]->initCArray();
	nodeList[0]->setParent(-5); //Parent is -5
	SDL_Rect startRect;
	startRect.h = nodeSize;
	startRect.w = nodeSize;
	startRect.x = startX-nodeSize/2;
	startRect.y = startY-nodeSize/2;
	SDL_SetRenderDrawColor(_renderer, 0, 255, 0, 255);
	SDL_RenderFillRect(_renderer, &startRect);

	//Finish point
	nodeList[1] = new node(endX, endY, 0, 10000, 1);
	nodeList[1]->initCArray();
	SDL_Rect finRect;
	finRect.h = nodeSize;
	finRect.w = nodeSize;
	finRect.x = endX-nodeSize/2;
	finRect.y = endY-nodeSize/2;
	SDL_SetRenderDrawColor(_renderer, 255, 0, 0, 255);
	SDL_RenderFillRect(_renderer, &finRect);

	//Blue
	SDL_SetRenderDrawColor(_renderer, 0, 0, 255, 255);

	//Fill the rest of the array with random nodes
	for (int i = 2; i < numNodes; i++) {
		int randX = rand() % _screenWidth;
		int randY = rand() % _screenHeight;
		if(!occupancyGrid[randX][randY]){
            a = new node(randX, randY, 0, 10000, i);
            nodeList[i] = a;
            nodeList[i]->initCArray();
            SDL_Rect nodeRect;
            nodeRect.h = nodeSize;
            nodeRect.w = nodeSize;
            nodeRect.x = randX-nodeSize/2;
            nodeRect.y = randY-nodeSize/2;
            SDL_RenderFillRect(_renderer, &nodeRect);
		}
		else{
            i=i-1;
		}
	}

	SDL_RenderPresent(_renderer);
	cout << "Populated nodes!" << endl;
}

//Sets position and draws obstacles. Can be called again to redraw
void MainGame::createObstacle() {

    SDL_SetRenderDrawColor(_renderer, 75, 0, 130, 255);
    for(int i = 0; i<sw; i++){
        for(int j = 0; j<sh; j++){
            if(i<350&&i>150&&j<200&&j>100){
                occupancyGrid[i][j]=true;
            }
        }
    }
    for(int i = 0; i<sw; i++){
        for(int j = 0; j<sh; j++){
            if(i<350&&i>150&&j<200&&j>100){
                if(occupancyGrid[i][j])SDL_RenderDrawPoint(_renderer,i,j);
            }
        }
    }
}

//Connects nodes to its neighbors
void MainGame::connect() {

	//Number of connections

    t1 = clock();
	SDL_SetRenderDrawColor(_renderer, 0, 0, 255, 255);
	//i = current node
	for (int i = 0; i < numNodes; i++) {
		//Check all nodes for possible connections (j=other nodes)

		for (int j = 0; j < numNodes; j++){
			//t1 = clock();
			//Don't connect a node to itself
			if (i == j) {}
			//Two different nodes:

			else {

				int dX = nodeList[i]->getxPos() - nodeList[j]->getxPos();
				int dY = nodeList[i]->getyPos() - nodeList[j]->getyPos();
				//If nodes are less than the max distance apart and are not blocked by an obstacle, connect and draw line

				if (static_cast<int>(sqrt(dX*dX + dY*dY)) < maxNodeDist){
					if(Line(nodeList[i]->getxPos(), nodeList[i]->getyPos(), nodeList[j]->getxPos(), nodeList[j]->getyPos())){
                        nodeList[i]->addConnection(j);
                        SDL_RenderDrawLine(_renderer, nodeList[i]->getxPos(), nodeList[i]->getyPos(), nodeList[j]->getxPos(), nodeList[j]->getyPos());
					}
				}



			}
            //t2 = clock();
            //timeTaken = timeTaken +(t2-t1)/(double) CLOCKS_PER_SEC*1000;
		}

	}
	SDL_RenderPresent(_renderer);
	cout << "Connected Nodes!" << endl;



	//Re-draw obstacles and start/finish
	createObstacle();
	createObstacle();
	redrawSF();
	redrawSF();

}

// Bresenham's line algorithm, taken from https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B
bool MainGame::Line(  float x1, float y1,  float x2,  float y2)
{


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
        //SetPixel(y,x, color);
        if(occupancyGrid[y][x]){
            return false;
        }
    }
    else
    {
        //SetPixel(x,y, color);
        if(occupancyGrid[x][y]){

            return false;
        }
    }

    error -= dy;
    if(error < 0)
    {
        y += ystep;
        error += dx;
    }
  }

  return true;
}

//Re-draw start and finish rectangles for clarity
void MainGame::redrawSF() {
	//cout<<"called redraw SF"<<endl;
	//Starting point
	SDL_Rect startRect;
	startRect.h = nodeSize;
	startRect.w = nodeSize;
	startRect.x = startX-nodeSize/2;
	startRect.y = startY - nodeSize / 2;
	SDL_SetRenderDrawColor(_renderer, 0, 255, 0, 255);
	SDL_RenderFillRect(_renderer, &startRect);

	//Finish point
	SDL_Rect finRect;
	finRect.h = nodeSize;
	finRect.w = nodeSize;
	finRect.x = endX - nodeSize / 2;
	finRect.y = endY - nodeSize / 2;
	SDL_SetRenderDrawColor(_renderer, 255, 0, 0, 255);
	SDL_RenderFillRect(_renderer, &finRect);

	SDL_RenderPresent(_renderer);
}

void MainGame::fillROSNodeArray(){
    for(int i = 0; i<numNodes; i++){
        beginner_tutorials::node curN;
        curN.id =nodeList[i]->getArrayValue();
        curN.x = nodeList[i]->getxPos();
        curN.y = nodeList[i]->getyPos();
        curN.connectionCounter = nodeList[i]->getConnectionCounter();

        int * cnA = nodeList[i]->getConnectionArray();

        for(int j = 0; j<numNodes; j++){
            curN.connections.push_back(cnA[j]);
        }
        //cout<<"Num connections: "<<curN.connections.size()<<endl;
        n.nodeLst.push_back(curN);
    }
    //cout<<numNodes<<n.nodeLst.size()<<endl;
}
