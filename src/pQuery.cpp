#include "pQuery.h"
#include <iostream>
#include <queue>
#include <SDL.h>
#include <chrono>
#include <ratio>
#include <ctime>
#include <vector>
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>


using namespace std;


//Number of nodes
int numNodes;

//Start and end positions
int startX = 120;
int startY = 130;
int endX = 400;
int endY = 130;

//Screen height and screen width
int sh;
int sw;

//Binary Occupancy map
vector<vector<int>> occupancyGrid;
nav_msgs::OccupancyGrid og;

//Size of node rectangles
const int nodeSize = 3;

//True after start and end have been selected, takes spacebar input
bool stillRunning = true;

//True at start, allows selection of start and end points
bool selectPts = true;

//Number of obstacles
const int numObs = 3;

//Max distance allowed between nodes
int maxNodeDist = 5000;

//Stores path from start to finish
string pathList;

//List of closed nodes
vector<int> closedNodeList;

//List of new connections to be added, cleared for each new node evaluated
vector<int> addedNodes;

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
	vector<int> connections;
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
	vector<int> getConnectionArray(){  return connections; }
	int getConnectionCounter(){  return connectionCounter;  }

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

		connections.resize(numNodes, -1);
	}



	//Adds connection to a neighboring node
	void addConnection(int c) {

	    if(connectionCounter<numNodes){
            connections[connectionCounter] = c;
            connectionCounter = connectionCounter + 1;
	    }
	    else{
            cout<<"out of bounds"<<endl;
            for(int i = 0; i<numNodes; i++){
                if(connections[i]==-2){
                    connections[i] = c;
                    cout<<"Found a spot"<<endl;
                    break;
                }

            }
	    }
	    if(c==-1)connectionCounter = connectionCounter-1;

	}

	void resetConnection(int c){
        //connections[c]=-2;
        for(int i = 0; i<2; i++){
            if(connections[i]==c){
                connections[i] = -2;
            }
        }
	}

    void resetCounter(){
        connectionCounter = 0;
    }

	//Prints all connections
	void printConnections() {
		for (int i = 0; i < numNodes; i++) {
			//if (connections[i] == -1) break;
			//else
				cout << connections[i]<<endl;
			//
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
vector<node*> nodeList;
//node* nodeList[numNodes];

//Initialize window parameters
pQuery::pQuery(int nN, int md, nav_msgs::OccupancyGrid ob)
{

    numNodes = nN;
    maxNodeDist = md;
    og = ob;
	_window = nullptr;
	_renderer = nullptr;
	sw = ob.info.width;
	sh = ob.info.height;
	_screenWidth = sw;
	_screenHeight = sh;
	_gameState = GameState::PLAY;


	occupancyGrid.resize(sw, vector<int>(sh, 0));
	nodeList.resize(numNodes,nullptr);
	closedNodeList.resize(numNodes,0);
	addedNodes.resize(numNodes,0);

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
    parseOGrid();
    createObstacle();
    createObstacle();
    SDL_RenderPresent(_renderer);
    SDL_RenderPresent(_renderer);



    cout << "Press space to query" << endl;


    fillLocalNodeArray();
    redrawSF();
    redrawSF();
	gameLoop();
    createObstacle();
	SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
	SDL_RenderClear(_renderer);
    SDL_DestroyWindow(_window);
    SDL_Quit();
    for(int i = 0; i<numNodes; i++) {
        //nodeList[i]->deleteCArray();
        delete nodeList[i];
    }




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



    //drawNodes();


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
	    //nodeList[aPos]->setConnectionCounter(g.connectionCounter);


	    for(std::vector<int>::const_iterator bit = g.connections.begin(); bit != g.connections.end(); ++bit)
        {
            int cn;
            cn = *bit;
            //cout<<cn<<endl;
            nodeList[aPos]->addConnection(cn);

        }
        //std::cout<<"Node "<<nodeList[aPos]->getArrayValue()<<" cc: "<<nodeList[aPos]->getConnectionCounter()<<std::endl;
        aPos = aPos+1;
    }
    setStartEnd();
    nodeList[0]->setParent(-5);
}

void pQuery::setStartEnd(){
    startX = sx;
    startY = sy;
    endX = ex;
    endY = ey;
    bool reconnectStart = false;
    bool reconnectFin = false;

    if (nodeList[0]->getxPos()!=sx||nodeList[0]->getyPos()!=sy){
        nodeList[0]->setXY(sx,sy);
        for(int i = 0;i<numNodes;i++){
            nodeList[i]->resetConnection(0);
        }
        nodeList[0]->initCArray();
        nodeList[0]->resetCounter();
        //cout<<"Reset connections successful"<<endl;
        //nodeList[0]->addConnection(13);
        //nodeList[13]->addConnection(0);
        reconnectStart = true;
    }


    if (nodeList[1]->getxPos()!=sx||nodeList[1]->getyPos()!=sy){
        nodeList[1]->setXY(ex,ey);
        //resetConnection(1);
        for(int i = 0;i<numNodes;i++){
            nodeList[i]->resetConnection(1);
        }
        nodeList[1]->initCArray();
        nodeList[1]->resetCounter();
        //cout<<"Reset connections successful"<<endl;
        reconnectFin = true;
    }
    if(reconnectStart){
        //cout<<"connecting 0"<<endl;
        connect(0);


    }
    if(reconnectFin){
        connect(1);
        //cout<<"connecting 1"<<endl;
    }

    //cout<<"Connect successful"<<endl;
    /*for(int i = 0;i<numNodes;i++){
        cout<<"Node "<<nodeList[i]->getArrayValue()<<":  x="<<nodeList[i]->getxPos()<<", y="<<nodeList[i]->getyPos()<<endl;
        nodeList[i]->printConnections();
    }*/



}

void pQuery::resetConnection(int nodeNumber){
    for(int i = 0; i<numNodes; i++){
        for(int j = 0; j<2; j++){
            if(nodeList[i]->getConnection(j) == nodeNumber) nodeList[i]->resetConnection(j);
        }
    }
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
							double time_elapsed = double(endClock - i4)/(double) CLOCKS_PER_SEC *1000;
							//cout << "Query time: "<< (endClock-i4)/(double) CLOCKS_PER_SEC*1000 <<endl;
							cout << "Time to calculate the route (ms): " << time_elapsed << endl;

							cout << "----------------------"<<endl;
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
            if(occupancyGrid[i][j])SDL_RenderDrawPoint(_renderer,i,j);



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
    //cout<<"Drew finish rectangle at y of: "<<finRect.y;
	SDL_RenderPresent(_renderer);
}



void pQuery::fillROSNodeArray(){
    for(int i = 0; i<numNodes; i++){
        beginner_tutorials::node curN;
        curN.id =nodeList[i]->getArrayValue();
        curN.x = nodeList[i]->getxPos();
        curN.y = nodeList[i]->getyPos();
        vector<int>cnA = nodeList[i]->getConnectionArray();

        for(int j = 0; j<numNodes; j++){
            curN.connections.push_back(cnA[j]);
        }

        n.nodeLst.push_back(curN);
    }
}

//Connects nodes to its neighbors
void pQuery::connect(int a) {

	//Number of connections


	SDL_SetRenderDrawColor(_renderer, 0, 0, 255, 255);
	//i = current node
	int i = a;
		//Check all nodes for possible connections (j=other nodes)


    for (int j = 0; j < numNodes; j++){
        //t1 = clock();
        //Don't connect a node to itself
        if (i == j) {  }
        //Two different nodes:
        else {
            int dX = nodeList[i]->getxPos() - nodeList[j]->getxPos();
            int dY = nodeList[i]->getyPos() - nodeList[j]->getyPos();
            //If nodes are less than the max distance apart and are not blocked by an obstacle, connect and draw line

            if (static_cast<int>(sqrt(dX*dX + dY*dY)) < maxNodeDist){
                if(Line(nodeList[i]->getxPos(), nodeList[i]->getyPos(), nodeList[j]->getxPos(), nodeList[j]->getyPos())){

                    nodeList[i]->addConnection(j);
                    nodeList[j]->addConnection(i);
                    //SDL_RenderDrawLine(_renderer, nodeList[i]->getxPos(), nodeList[i]->getyPos(), nodeList[j]->getxPos(), nodeList[j]->getyPos());



                }
            }
        }
            //t2 = clock();
            //timeTaken = timeTaken +(t2-t1)/(double) CLOCKS_PER_SEC*1000;
    }

    //SDL_RenderPresent(_renderer);







}

// Bresenham's line algorithm, taken from https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B
bool pQuery::Line(  float x1, float y1,  float x2,  float y2)
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
				if (nodeList[i]->getParent() == cxn || cxn == -2) {
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

void pQuery::drawNodes(){
     for(int i = 0; i<numNodes; i++){
        SDL_Rect nodeRect;
        nodeRect.h = nodeSize;
        nodeRect.w = nodeSize;
        nodeRect.x = nodeList[i]->getxPos()-nodeSize/2;
        nodeRect.y = nodeList[i]->getyPos()-nodeSize/2;
        SDL_RenderFillRect(_renderer, &nodeRect);

     }
     SDL_RenderPresent(_renderer);
}

void pQuery::parseOGrid(){
    int oCounter = 0;
    for(int j = 0; j<sh; j++){
        for(int i = 0; i<sw; i++){
            occupancyGrid[i][j] = og.data[oCounter];
            oCounter++;

        }
    }
}
