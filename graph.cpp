/*
priority queue, graph & Dijkstra's algorithm - 4/4/2020

instructions:
- a graph class with random edges and costs
- implement a priority queue class
- implement Dijkstra's algorithm for shortest paths
- compute an average shortest path cost (from a source node)

reference:
- Wikipedia: Dijkstra's algorithm

explanation and code output:
- see graph.txt
*/
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
using namespace std;

template <class T>
ostream& operator<<(ostream& out, const vector<T>& vec){
    // print elements of a vector
    if(vec.size()==0) out << "[ ]";
    else{
        out << "[ ";
        for(int i=0; i<vec.size()-1; i++) out << vec[i] << ", ";
        out << vec[vec.size()-1] << " ]";
    }
    return out;
}

inline double prob(){return static_cast<double>(rand())/RAND_MAX;}
inline double uniformRand(double min, double max){return min+(max-min)*prob();}

/******************************************************************************/

struct node{
    // queue element
    int label;
    double value;
    node* next;
    node(int label, double value):label(label),value(value),next(0){}
};

inline ostream& operator<<(ostream& out, const node& n){
    out << '(' << n.label << ',' << n.value << ')';
    return out;
}

class priorityQueue{
private:
    node* head;
public:
    /**** constructors ****/
    priorityQueue();
    /**** destructors ****/
    // ~priorityQueue();
    /**** accessors ****/
    int getSize();
    bool isEmpty();
    bool contains(int label);
    node peekHeadNode();
    node peekTailNode();
    void print();
    /**** mutators ****/
    void add(int label, double value);
    void add(node n);
    void del(int label);
    void chgPriority(int label, double value);
    node popHeadNode();
};

/**** constructors ****/

priorityQueue::priorityQueue():head(0){}

/**** accessors ****/

int priorityQueue::getSize(){
    // number of nodes in queue
    int size = 0;
    node* p = head;
    while(p!=0){
        size++;
        p = p->next;
    }
    return size;
}

bool priorityQueue::isEmpty(){
    // check if queue is empty
    if(head==0) return true;
    return false;
}

bool priorityQueue::contains(int label){
    // check if queue contains node as labelled
    node* p = head;
    while(p!=0){
        if(p->label==label) return true;
        p = p->next;
    }
    return false;
}

node priorityQueue::peekHeadNode(){
    // return a copy of head node
    node n(head->label,head->value);
    return n;
}

node priorityQueue::peekTailNode(){
    // return a copy of tail node
    node* p = head;
    while(p->next!=0) p = p->next;
    node n(p->label,p->value);
    return n;
}

void priorityQueue::print(){
    // print whole queue
    node* p = head;
    while(p!=0){
        cout << *p << " -> ";
        p = p->next;
    }
    cout << "###" << endl;
}

/**** mutators ****/

void priorityQueue::add(int label, double value){
    // add new node (label,value)
    node* n = new node(label,value);
    node* p;
    if(head==0 || value>head->value){
        // empty queue or higher priority than head node
        n->next = head;
        head = n;
    }else{
        p = head;
        // stop at node with priority just higher than n->value
        while(p->next!=0 && p->next->value>=value)
            p = p->next;
        n->next = p->next;
        p->next = n;
    }
}

void priorityQueue::add(node n){
    // add new node
    add(n.label,n.value);
}

void priorityQueue::del(int label){
    // delete node as labelled
    node* p = head;
    node* prev;
    if(contains(label)){
        while(p->label!=label){
            prev = p;
            p = p->next;
        }
        if(p==head) head = head->next;
        else prev->next = p->next;
        free(p);
    }
}

void priorityQueue::chgPriority(int label, double value){
    // change priority of node as labelled
    del(label);
    add(label,value);
}

node priorityQueue::popHeadNode(){
    // pop and return a copy of head node
    node* p = head;
    head = head->next;
    node n(p->label,p->value);
    free(p);
    return n;
}

/******************************************************************************/

class Graph{
private:
    int size;
    double minCost,maxCost;
    double** cost; // edge cost matrix
    bool** graph; // graph edge matrix (aka. connectivity/adjacency matrix)
public:
    /**** constructors ****/
    Graph(int size, double density, double minCost, double maxCost);
    /**** destructor ****/
    // ~Graph();
    /**** accessors ****/
    int getSize();
    int getEdges();
    void printGraph(bool matrix);
    bool isAdjacent(int n, int m);
    vector<int> getNeighbors(int n);
    double getCost(int n, int m);
    void printShortestPaths(int n);
    double getAvgShortestPathCost(int n);
    /**** mutators ****/
    void addEdge(int n, int m);
    void deleteEdge(int n, int m);
    void setCost(int n, int m, double c);
    void setRandCost(int n, int m);
    /**** algorithms ****/
    bool isConnected();
    vector<node>* shortestPaths(int n);
};

/**** constructors ****/

Graph::Graph(int size, double density, double minCost, double maxCost){
    // random undirected graph constructor
    this->size = size;
    this->minCost = minCost;
    this->maxCost = maxCost;
    /**** graph edge matrix ****/
    graph = new bool*[size];
    for(int i=0; i<size; i++)
        graph[i] = new bool[size];
    // symmetric matrix
    for(int i=0; i<size; i++)
        for(int j=i; j<size; j++)
            if(i==j) graph[i][j] = false; // no self-cycle
            else graph[i][j] = graph[j][i] = (prob()<density);
    /**** edge cost matrix ****/
    cost = new double*[size];
    for(int i=0; i<size; i++)
        cost[i] = new double[size];
    // symmetric matrix
    for(int i=0; i<size; i++)
        for(int j=i; j<size; j++)
            if(i==j) cost[i][j] = 0;
            else cost[i][j] = cost[j][i] = uniformRand(minCost,maxCost);
}

/**** accessors ****/

int Graph::getSize(){
    // number of nodes (aka vertices)
    return size;
}

int Graph::getEdges(){
    // number of edges
    int edges = 0;
    for(int i=0; i<size; i++)
        for(int j=i+1; j<size; j++)
            edges += graph[i][j];
    return edges;
}

void Graph::printGraph(bool matrix=true){
    if(matrix){
        // print adjacency matrix
        cout << string(2*size-1,'-') << endl;
        for(int i=0; i<size; i++){
            for(int j=0; j<size; j++)
                cout << graph[i][j] << ' ';
            cout << endl;
        }
        cout << string(2*size-1,'-') << endl;
    }else{
        // print neighbor lists
        cout << string(2*size-1,'-') << endl;
        for(int i=0; i<size; i++){
            cout << i << ": ";
            for(int j=0; j<size; j++)
                if(graph[i][j]==1) cout << j << ' ';
            cout << endl;
        }
        cout << string(2*size-1,'-') << endl;
    }
}

bool Graph::isAdjacent(int n, int m){
    // check if node n and m are connected
    return graph[n][m];
}

vector<int> Graph::getNeighbors(int n){
    // neighbors of node n
    vector<int> nbors;
    for(int i=0; i<size; i++)
        if(graph[n][i]==1) nbors.push_back(i);
    return nbors;
}

double Graph::getCost(int n, int m){
    // cost from node n to m
    return cost[n][m];
}

void Graph::printShortestPaths(int n){
    // print shortest paths from source node n to other nodes
    vector<node>* paths = shortestPaths(n); // array of shortest paths
    for(int i=0; i<size; i++){
        if(i==n) cout << "(source node) ";
        else if(paths[i].size()==0) cout << "(disconnected) ";
        cout << "shortest path from node " << n << " to node " << i << ":\n  ";
        cout << paths[i] << endl;
    }
}

double Graph::getAvgShortestPathCost(int n){
    // compute average shortest path cost from source node n
    vector<node>* paths = shortestPaths(n);
    double totalCost = 0;
    int connectedNodes = 0;
    for(int i=0; i<size; i++){
        if(paths[i].size()>0){ // there is a path that leads from node n to i
            connectedNodes++;
            totalCost += paths[i][paths[i].size()-1].value;
        }
    }
    return totalCost/connectedNodes;
}

/**** mutators ****/

void Graph::addEdge(int n, int m){
    // connect node n and m
    graph[n][m] = graph[m][n] = 1;
}

void Graph::deleteEdge(int n, int m){
    // disconnect node n and m
    graph[n][m] = graph[m][n] = 0;
    cost[n][m] = cost[m][n] = 0;
}

void Graph::setCost(int n, int m, double c){
    // set cost from node n to m as c
    cost[n][m] = cost[m][n] = c;
}

void Graph::setRandCost(int n, int m){
    // randomly set cost from node n to m
    cost[n][m] = cost[m][n] = uniformRand(minCost,maxCost);
}

/**** algorithms ****/

bool Graph::isConnected(){
    // check if graph is connected
    int oldSize, closedSize=0;
    bool* closed = new bool[size]; // expanded nodes
    bool* open = new bool[size]; // nodes to be expanded
    for(int i=0; i<size; i++)
        closed[i] = open[i] = false;
    open[0] = true; // initialization: start from node 0
    while(closedSize<size){
        oldSize = closedSize;
        for(int i=0; i<size; i++){
            if(open[i] && !closed[i]){ // new nodes to expand
                closed[i] = true;
                closedSize++;
                for(int j=0; j<size; j++)
                    open[j] = open[j]||graph[i][j];
            }
            // for(int i=0; i<size; i++) cout << closed[i] << ' ';
            // cout << "\n";
            // for(int i=0; i<size; i++) cout << open[i] << ' ';
            // cout << "\n\n";
        }
        if(oldSize==closedSize) return false;
    }
    return true;
}

vector<node>* Graph::shortestPaths(int n){
    // Dijkstra's shortest path from source node n to other nodes
    // return an array of shortest path vectors (of nodes)
    const int inf = size*maxCost;
    vector<node>* paths = new vector<node>[size]; // shortest paths
    int current; // node currently being explored
    int* prev = new int[size];
    bool* closed = new bool[size]; // expanded nodes
    double* nodeCost = new double[size]; // tentative cost from node n
    double c; // tentative cost
    priorityQueue q; // priority queue for uniform cost search

    /**** initialization ****/
    nodeCost[n] = 0; // start from node n
    for(int i=0; i<size; i++){
        closed[i] = false;
        prev[i] = -1;
        if(i!=n) nodeCost[i] = inf;
        // prioirty: -cost
        node tmpNode(i,-nodeCost[i]);
        q.add(tmpNode);
    }
    // q.print();

    /**** uniform cost search ****/
    while(!q.isEmpty() && q.peekHeadNode().value!=-inf){
        current = q.popHeadNode().label; // pop node with lowest cost
        for(int i=0; i<size; i++)
            // unvisited neighbors of current node
            if(!closed[i] && graph[current][i]){
                c = nodeCost[current]+cost[current][i];
                // update tentative cost
                if(c<nodeCost[i]){
                    nodeCost[i] = c;
                    prev[i] = current;
                    q.chgPriority(i,-c); // update priority queue
                }
            }
        closed[current] = true;
        // q.print(); // show the search
    }

    // for(int i=0; i<size; i++) cout << prev[i] << ' ';
    // cout << endl;

    /**** path from node n to other nodes ****/
    for(int m=0; m<size; m++){
        if(prev[m]!=-1){ // node m connected to n
            int i = m; // iterator from node m to n
            while(i!=n){ // continue if have not traced back to node n
                node tmpNode(i,nodeCost[i]);
                paths[m].insert(paths[m].begin(),tmpNode);
                i = prev[i];
            }
            node tmpNode(n,nodeCost[n]);
            paths[m].insert(paths[m].begin(),tmpNode); // prepend source node n
        }
    }
    return paths;
}

/******************************************************************************/

int main(){
    srand(0);
    const int source=0; // source node shortest path search
    const int size=50; // number of nodes
    const double density=.2, minCost=1, maxCost=10; // density and cost range
    // const double density=.4, minCost=1, maxCost=10;
    Graph g(size,density,minCost,maxCost); // construct graph
    // g.printGraph(); // print adjacency matrix
    cout << "connected: " << g.isConnected() << endl; // check connectedness
    cout << "------------------------" << endl;
    g.printShortestPaths(source); // print shortest paths from source node to other nodes
    cout << "------------------------" << endl;
    cout << "avg shortest path cost: " << g.getAvgShortestPathCost(source) << endl; // compute average shortest path cost
    return 0;
}
