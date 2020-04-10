/*
graph, shortest path search & minimum spanning tree - 7/4/2020
*/
#include "graph.cpp"
using namespace std;

int main(){
    const int source=0; // source node shortest path search
    ifstream graphFile("mstGraph.txt"); // read in graph input file
    Graph g(graphFile); // construct graph
    graphFile.close();

    g.printGraph(); // print adjacency matrix
    cout << "connected: " << g.isConnected() << endl; // check connectedness
    seperator();
    g.printShortestPaths(source); // print shortest paths from source node to other nodes
    seperator();
    cout << "avg shortest path cost: " << g.getAvgShortestPathCost(source) << endl; // compute average shortest path cost
    seperator();
    g.printMinSpanningTree(0); // print edges & cost of minimum spanning tree
}
