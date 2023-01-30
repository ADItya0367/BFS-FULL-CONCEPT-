# BFS-FULL-CONCEPT-
BREADTH FIRST SEARCH -is discussed in this README File where you will be getting the full concept of theory and the coding part that how actually BFS is implemented

CONCEPTS DISCUSSED ARE AS FOLLOWS-
** APPLICATIONS Of BFS
** implementation uses an adjacency list representation of graphs
**implementation for BFS traversal for the entire graph



The breadth-first search (BFS) algorithm is used to search a tree or graph data structure for a node that meets a set of criteria. It starts at the tree’s root or graph and searches/visits all nodes at the current depth level before moving on to the nodes at the next depth level. Breadth-first search can be used to solve many problems in graph theory.

Breadth-First Traversal (or Search) for a graph is similar to the Breadth-First Traversal of a tree (See method 2 of this post). 

The only catch here is, that, unlike trees, graphs may contain cycles, so we may come to the same node again. To avoid processing a node more than once, we divide the vertices into two categories:

Visited and
Not visited.
A boolean visited array is used to mark the visited vertices. For simplicity, it is assumed that all vertices are reachable from the starting vertex. BFS uses a queue data structure for traversal.

--------------------------------------------
Follow the below method to implement BFS traversal.

Declare a queue and insert the starting vertex.
Initialize a visited array and mark the starting vertex as visited.
Follow the below process till the queue becomes empty:
Remove the first vertex of the queue.
Mark that vertex as visited.
Insert all the unvisited neighbors of the vertex into the queue.

--------------------------------------------
The implementation uses an adjacency list representation of graphs. STL‘s list container stores lists of adjacent nodes and the queue of nodes needed for BFS traversal.
--------------------------------------------
CODE 


// Program to print BFS traversal from a given
// source vertex. BFS(int s) traverses vertices
// reachable from s.
#include <bits/stdc++.h>
using namespace std;
 
// This class represents a directed graph using
// adjacency list representation
class Graph {
    int V; // No. of vertices
 
    // Pointer to an array containing adjacency
    // lists
    vector<list<int> > adj;
 
public:
    Graph(int V); // Constructor
 
    // function to add an edge to graph
    void addEdge(int v, int w);
 
    // prints BFS traversal from a given source s
    void BFS(int s);
};
 
Graph::Graph(int V)
{
    this->V = V;
    adj.resize(V);
}
 
void Graph::addEdge(int v, int w)
{
    adj[v].push_back(w); // Add w to v’s list.
}
 
void Graph::BFS(int s)
{
    // Mark all the vertices as not visited
    vector<bool> visited;
    visited.resize(V, false);
 
    // Create a queue for BFS
    list<int> queue;
 
    // Mark the current node as visited and enqueue it
    visited[s] = true;
    queue.push_back(s);
 
    while (!queue.empty()) {
        // Dequeue a vertex from queue and print it
        s = queue.front();
        cout << s << " ";
        queue.pop_front();
 
        // Get all adjacent vertices of the dequeued
        // vertex s. If a adjacent has not been visited,
        // then mark it visited and enqueue it
        for (auto adjecent : adj[s]) {
            if (!visited[adjecent]) {
                visited[adjecent] = true;
                queue.push_back(adjecent);
            }
        }
    }
}
 
// Driver program to test methods of graph class
int main()
{
    // Create a graph given in the above diagram
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 0);
    g.addEdge(2, 3);
    g.addEdge(3, 3);
 
    cout << "Following is Breadth First Traversal "
         << "(starting from vertex 2) \n";
    g.BFS(2);
 
    return 0;
}

Time Complexity: O(V+E), where V is the number of nodes and E is the number of edges.
Auxiliary Space: 
  ------------------------------------------------------------------------------------------------------------------------------------------------------------------

BFS for Disconnected Graph:
Note that the above code traverses only the vertices reachable from a given source vertex. In every situation, all the vertices may not be reachable from a given vertex (i.e. for a disconnected graph). 


Below is the implementation for BFS traversal for the entire graph (valid for directed as well as undirected graphs) with possible multiple disconnected components:


CODE 

/*
-> Generic Function for BFS traversal of a Graph
 (valid for directed as well as undirected graphs
 which can have multiple disconnected components)
-- Inputs --
-> V - represents number of vertices in the Graph
-> adj[] - represents adjacency list for the Graph
-- Output --
-> bfs_traversal - a vector containing bfs traversal
for entire graph
*/
 
vector<int> bfsOfGraph(int V, vector<int> adj[])
{
    vector<int> bfs_traversal;
    vector<bool> vis(V, false);
    for (int i = 0; i < V; ++i) {
 
        // To check if already visited
        if (!vis[i]) {
            queue<int> q;
            vis[i] = true;
            q.push(i);
 
            // BFS starting from ith node
            while (!q.empty()) {
                int g_node = q.front();
                q.pop();
                bfs_traversal.push_back(g_node);
                for (auto it : adj[g_node]) {
                    if (!vis[it]) {
                        vis[it] = true;
                        q.push(it);
                    }
                }
            }
        }
    }
    return bfs_traversal;
}

--------------------------------------------

APPLICATIONS OF BFS ARE AS FOLLOWS-

1-Shortest Path and Minimum Spanning Tree for unweighted graph In an unweighted graph, the shortest path is the path with least number of edges. With Breadth First, we always reach a vertex from given source using the minimum number of edges. Also, in case of unweighted graphs, any spanning tree is Minimum Spanning Tree and we can use either Depth or Breadth first traversal for finding a spanning tree. 
2-Peer to Peer Networks. In Peer to Peer Networks like BitTorrent, Breadth First Search is used to find all neighbor nodes. 
3-Crawlers in Search Engines: Crawlers build index using Breadth First. The idea is to start from source page and follow all links from source and keep doing same. 4-Depth First Traversal can also be used for crawlers, but the advantage with Breadth First Traversal is, depth or levels of the built tree can be limited. 
5-Social Networking Websites: In social networks, we can find people within a given distance ‘k’ from a person using Breadth First Search till ‘k’ levels. 
6-GPS Navigation systems: Breadth First Search is used to find all neighboring locations. 
7-Broadcasting in Network: In networks, a broadcasted packet follows Breadth First Search to reach all nodes. 
8-In Garbage Collection: Breadth First Search is used in copying garbage collection using Cheney’s algorithm. Refer this and for details. Breadth First Search is preferred over Depth First Search because of better locality of reference: 
9-Cycle detection in undirected graph: In undirected graphs, either Breadth First Search or Depth First Search can be used to detect cycle. We can use BFS to detect cycle in a directed graph also,
10-Ford–Fulkerson algorithm In Ford-Fulkerson algorithm, we can either use Breadth First or Depth First Traversal to find the maximum flow. Breadth First Traversal is preferred as it reduces worst case time complexity to O(VE2). 
11-To test if a graph is Bipartite We can either use Breadth First or Depth First Traversal. 
12-Path Finding We can either use Breadth First or Depth First Traversal to find if there is a path between two vertices. 
13-Finding all nodes within one connected component: We can either use Breadth First or Depth First Traversal to find all nodes reachable from a given node. 
14-AI: In AI, BFS is used in traversing a game tree to find the best move.
15-Network Security: In the field of network security, BFS is used in traversing a network to find all the devices connected to it.
16-Undirected graph: Finding all connected components in an undirected graph.
17-Topological sorting: BFS can be used to find a topological ordering of the nodes in a directed acyclic graph (DAG).
18-Image processing: BFS can be used to flood fill an image with a particular color or to find connected components of pixels.
19-Recommender systems: BFS can be used to find similar items in a large dataset by traversing the items’ connections in a similarity graph.
