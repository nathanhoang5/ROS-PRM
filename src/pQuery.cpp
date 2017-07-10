#include "pQuery.h"
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
const int numNodes = 25;

//Start and end positions
int startX = 120;
int startY = 130;
int endX = 400;
int endY = 130;

//Screen height and screen width
const int sh = 300;
const int sw = 500;

//Binary Occupancy map
int occupancyGrid[sw][sh];

//Size of node rectangles
const int nodeSize = 3;

//True after start and end have been selected, takes spacebar input
bool stillRunning = true;

//True at start, allows selection of start and end points
bool selectPts = true;

//Number of obstacles
const int numObs = 3;

//Array storing all obstacles
SDL_Rect* obs = new SDL_Rect[numObs];

//Max distance allowed between nodes
const int maxNodeDist = 5000;

//Stores path from start to finish
string pathList;

//List of closed nodes
int * closedNodeList = new int [numNodes];

//List of new connections to be added, cleared for each new node evaluated
int * addedNodes = new int[numNodes];

//Counter for index of closed node array
int closedCounter = 0;

//Counter for index of added nodes array
int aNCounter = 0;

double runTime = 0;

//beginner_tutorials::nodeArray n;

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
	int* getConnectionArray(){  return connections; }

    void setXY(int x, int y){
        xPos = x;
        yPos = y;
    }

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
node* nodeList[numNodes];

//Initialize window parameters
pQuery::pQuery()
{
	_window = nullptr;
	_renderer = nullptr;
	_screenWidth = sw;
	_screenHeight = sh;
	_gameState = GameState::PLAY;
	tester = 35;
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
pQuery::~pQuery()
{

}


//Called from main class, starts application depending on if selectPoints==true, then starts gameLoop()
void pQuery::run() {
	initSystems();

    createObstacle();
    createObstacle();
    SDL_RenderPresent(_renderer);
    SDL_RenderPresent(_renderer);



    cout << "Select start point" << endl;



	gameLoop();

	SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
	SDL_RenderClear(_renderer);
    SDL_DestroyWindow(_window);
    SDL_Quit();

	//cout<<"GameState = EXIT"<<endl;
}

//Create window and initialize lists, makes white background
void pQuery::initSystems() {

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

	selectPts = false;
    stillRunning = true;

    fillLocalNodeArray();
    redrawSF();
    redrawSF();

}

void pQuery::fillLocalNodeArray(){
    static node* a;
    int aPos = 0;
    for(std::vector<beginner_tutorials::node>::const_iterator it = n.nodeLst.begin(); it != n.nodeLst.end(); ++it)
    {
	    beginner_tutorials::node g;
	    g = *it;

	    nodeList[aPos] = new node(g.x, g.y, 0, 10000, g.id);
	    nodeList[aPos]->initCArray();
	    //std::cout<<g.id<<std::endl;

	    for(std::vector<int>::const_iterator bit = g.connections.begin(); bit != g.connections.end(); ++bit)
        {
            int cn;
            cn = *bit;
            //cout<<cn<<endl;
            nodeList[aPos]->addConnection(cn);

        }

        aPos = aPos+1;
    }
    //setStartEnd();
    nodeList[0]->setParent(-5);
}

void pQuery::setStartEnd(){
    nodeList[0]->setXY(sx,sy);
    nodeList[1]->setXY(ex,ey);
    startX = sx;
    startY = sy;
    endX = ex;
    endY = ey;
}

//If there is no error, continue to process input
void pQuery::gameLoop() {
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
void pQuery::processInput() {
	SDL_Event evnt;
	while (SDL_PollEvent(&evnt) == true) {
		switch (evnt.type) {
            //If exit is clicked, close application
            case SDL_QUIT:
				_gameState = GameState::EXIT;
				break;

			//case SDL_MOUSEMOTION:
			//	cout << evnt.motion.x << " " << evnt.motion.y << endl;

            //If spacebar pressed
			case SDL_KEYDOWN:
				if (stillRunning) {
					if (evnt.key.keysym.scancode == SDL_SCANCODE_SPACE) {

                        if (counter == 0) {
							i4 = clock();
							query();
							endClock = clock();
							//Adds time taken for each section and print
							double time_elapsed = (double(i1 - start) + double(i3 - i2) + double(endClock - i4))/(double) CLOCKS_PER_SEC *1000;
							//cout << "Query time: "<< (endClock-i4)/(double) CLOCKS_PER_SEC*1000 <<endl;
							cout << "Time to calculate the route (ms): " << time_elapsed << endl;
							//cout << "Clocks per second: " << (double) CLOCKS_PER_SEC<<endl;
							//End program
							runTime = time_elapsed;
							stillRunning = false;

							break;
						}
					}
				}
		}
	}
}

void pQuery::createObstacle() {

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

//Re-draw start and finish rectangles for clarity
void pQuery::redrawSF() {
	//cout<<"called redraw SF"<<endl;
	//Starting point
	SDL_Rect startRect;
	startRect.h = nodeSize;
	startRect.w = nodeSize;
	startRect.x = nodeList[0]->getxPos()-nodeSize/2;
	startRect.y = nodeList[0]->getyPos() - nodeSize / 2;
	SDL_SetRenderDrawColor(_renderer, 0, 255, 0, 255);
	SDL_RenderFillRect(_renderer, &startRect);

	//Finish point
	SDL_Rect finRect;
	finRect.h = nodeSize;
	finRect.w = nodeSize;
	finRect.x = nodeList[1]->getxPos() - nodeSize / 2;
	finRect.y = nodeList[1]->getyPos() - nodeSize / 2;
	SDL_SetRenderDrawColor(_renderer, 255, 0, 0, 255);
	SDL_RenderFillRect(_renderer, &finRect);

	SDL_RenderPresent(_renderer);
}



void pQuery::fillROSNodeArray(){
    for(int i = 0; i<numNodes; i++){
        beginner_tutorials::node curN;
        curN.id =nodeList[i]->getArrayValue();
        curN.x = nodeList[i]->getxPos();
        curN.y = nodeList[i]->getyPos();
        int * cnA = nodeList[i]->getConnectionArray();

        for(int j = 0; j<numNodes; j++){
            curN.connections.push_back(cnA[j]);
        }

        n.nodeLst.push_back(curN);
    }
}

//Priority queue to store open (activated) nodes
priority_queue<node, vector<node>, CompareNode> pq = priority_queue<node, vector<node>, CompareNode>();

//Finds the best path!
void pQuery::query() {

	//True when path is found
	bool found = false;
	//Add starting node to pq
	pq.push(*nodeList[0]);
	//While the path hasn't been found...
	while (found == false) {

		//If there are no more open nodes, there is no possible path
		if (pq.size() == 0) {
			cout<<"No path to end!"<<endl;
			gameLoop();
		}
		//The current node being evaluated
		int i = pq.top().getArrayValue();
			//j is index in the current node's connection list
			for (int j = 0; j < numNodes; j++) {
				//One of current node's connections, will be -1 if node has no more connections
				int cxn = nodeList[i]->getConnection(j);
				//Do nothing if the connection is its parent
				if (nodeList[i]->getParent() == cxn) {
				}
				//If the connection is the finish, break out of loop and display path
				else if (cxn == 1) {
					//cout << "Found path!" << endl;
					clearQueueList();
					foundNode(i);

					found = true;
					break;
				}
				//If the connection has not been found add node to the open list
				else if ((!(cxn == -1))&&notFound(cxn)) {
					addedNodes[aNCounter] = cxn;
					aNCounter = aNCounter + 1;
				}
				//If the connection has been found check to see if path from current node is shorter
				else if ((!(cxn == -1)) && (notFound(cxn)==false)) {
					//If it is reassign parent
					if(getMoveDist(i, cxn)<nodeList[cxn]->getLevel()){
						nodeList[cxn]->setParent(i);
						nodeList[cxn]->setPriority(getMoveDist(i, cxn));
					}
				}
				//If the end of the connection list has been reached
 				else if(cxn == -1){
					//Remove current node from pq
					pq.pop();
					//Add all found connections to pq
					addNodes(i);
					break;
				}
			}
	}

}

//Return total distance from START node to node b through path of node a
int pQuery::getMoveDist(int a, int b) {
	int dX = nodeList[a]->getxPos() - nodeList[b]->getxPos();
	int dY = nodeList[a]->getyPos() - nodeList[b]->getyPos();
	int d =(sqrt(dX*dX + dY*dY));
	return nodeList[a]->getLevel()+d;
}

//If node has been found (a=array value of last node connected to finish node)
void pQuery::foundNode(int a) {

	//Clear all lines
	//redrawFin();
	//Current node in path (starts from the end)
	int curNode = a;

	//Green
	SDL_SetRenderDrawColor(_renderer, 22, 204, 28, 255);
	//Record first node in path

	pathList += to_string(nodeList[curNode]->getArrayValue());
	pathList += " ";


	//Draw line from finish to connecting node
	SDL_RenderDrawLine(_renderer, nodeList[curNode]->getxPos(), nodeList[curNode]->getyPos(), nodeList[1]->getxPos(), nodeList[1]->getyPos());
	//While not at the start node
	while (!(nodeList[curNode]->getParent() == -5)) {

        //cout<<"Array value: "<<nodeList[curNode]->getArrayValue()<<endl;
        //cout<<"Parent: "<<nodeList[curNode]->getParent()<<endl;

        if(nodeList[curNode]->getArrayValue()==nodeList[curNode]->getParent()){
            //cout<<"parent value same as array value"<<endl;
            //cout<<"PosX: " << nodeList[curNode]->getxPos()<<endl;
            //cout<<"PosY: " << nodeList[curNode]->getyPos()<<endl;
            break;
        }
		//Add the current node to the path record and draw line
		pathList += to_string(nodeList[curNode]->getParent());
		pathList += " ";
		SDL_RenderDrawLine(_renderer, nodeList[curNode]->getxPos(), nodeList[curNode]->getyPos(), nodeList[nodeList[curNode]->getParent()]->getxPos(), nodeList[nodeList[curNode]->getParent()]->getyPos());
		//Find the parent of the current node
		curNode = nodeList[curNode]->getParent();
	}
	//Display path record and draw path
	//cout << "Path: " << pathList << endl;
	//cout <<"PQ Size Before: "<<pq.size()<<endl;
	pq = priority_queue<node, vector<node>, CompareNode>();
	//cout <<"PQ Size After: "<<pq.size()<<endl;
	closedCounter = 0;
	//cout <<"Closed Counter: "<<closedCounter;
	for(int i = 0; i<numNodes; i++){
            nodeList[i] = nullptr;

	}
	pathList = "";
	SDL_RenderPresent(_renderer);

}

//Returns true if node is not on the open nodes list(pq) or closed node list
bool pQuery::notFound(int a) {
	for (int i = 0; i < numNodes; i++) {
		if (closedNodeList[i] == a) {
			return false;
		}
		if (closedNodeList[i] == -1) {
			return true;
		}
	}
	return true;
}

//Adds nodes to the open node list (a=parent node)
void pQuery::addNodes(int a) {
	for (int i = 0; i < numNodes; i++) {
		//If there are no more added nodes, clear queue waitlist (addedNodes[]) and break
		if (addedNodes[i] == -1) {
			clearQueueList();
			break;
		}
		//Initialize node (set parent and priority), add to pq, and add to closed node list
		else {
			nodeList[addedNodes[i]]->setParent(a);
			nodeList[addedNodes[i]]->setPriority(getMoveDist(a, addedNodes[i]));
			pq.push(*nodeList[addedNodes[i]]);
			closedNodeList[closedCounter] = addedNodes[i];
			closedCounter = closedCounter + 1;
		}
	}
}

//Reset addedNodes[]
void pQuery::clearQueueList() {
	aNCounter = 0;
	for (int i = 0; i < numNodes; i++) {
		addedNodes[i] = -1;
	}
}
