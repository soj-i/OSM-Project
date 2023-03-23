/*main.cpp*/

//
// Program to input Nodes (positions), Buildings and Footways
// from an Open Street Map file.
// 


#include <iostream>
#include <string>
#include <iomanip>
#include <limits>
#include <cmath>
#include <queue>

#include "building.h"
#include "buildings.h"
#include "footway.h"
#include "footways.h"
#include "node.h"
#include "nodes.h"
#include "osm.h"
#include "tinyxml2.h"
#include "graph.h"
#include "dist.h"

using namespace std;
using namespace tinyxml2;


//
// traverses nodes and footways, building a graph
//
void graphBuilder(graph& g, Nodes& n, Footways& fw)
{
  for(map<long long, Node>::iterator start = n.begin(); start != n.end(); start++)
  {
    g.addVertex(start->first);
  }

  for (auto& footway : fw.MapFootways)
  {
    for(size_t s = 0; s < footway.NodeIDs.size() - 1; s++)
    {
      long long from = footway.NodeIDs[s];
      long long end = footway.NodeIDs[s + 1];

      double lat1, lat2, lon1, lon2;
      bool entrance;

      n.find(from, lat1, lon1, entrance);
      n.find(end, lat2, lon2, entrance);

      double d1 = distBetween2Points(lat1, lon1, lat2, lon2);
      double d2 = distBetween2Points(lat2, lon2, lat1, lon1);

      g.addEdge(from, end, d1);
      g.addEdge(end, from, d2);
    }
  }
}

constexpr double INF = numeric_limits<double>::max();

//
// Simplifies navigation function by finding and printing
// the closest footway ID, node ID, and Distance
//
long long navigationhelper(Footways& footways, Nodes& nodes, double lat1, double lon1)
{
  double min = INF;
  long long node_id;
  long long foot_id;

  for (auto& foot: footways.MapFootways)
  {
    for (size_t i = 0; i < foot.NodeIDs.size(); i++ )
    {
      double lat2;
      double lon2;
      bool isEntrance;

      nodes.find(foot.NodeIDs[i], lat2, lon2, isEntrance);
      double d2 = distBetween2Points(lat1, lon1, lat2, lon2);

      if( d2 < min)
      {
        min = d2;
        foot_id = foot.ID;
        node_id = foot.NodeIDs[i];
      }
    }
  }
  cout << "  Closest footway ID " << foot_id << ", node ID " << node_id << ", distance " << min << endl;
  return node_id;

}

//
// Mr. Dijkstra's algorithm to find the shortest path between vertices 
//

vector<long long> Dijkstra(graph& G, //graph, vertex, distance map, pred map
  long long startV, 
  map<long long, double>& distances, map<long long, long long>& pred)
{
   priority_queue<pair <double, long long>, vector<pair <double, long long>>, greater <pair <double, long long>>> unvisitedQueue;
   
   map<long long, bool> visited;
   vector<long long> orderVisited;
  
   
   for (long long vtx: G.getVertices())
   {
      distances[vtx] = (vtx == startV) ? 0 : INF;
      pred[vtx] = -1;
   }
   
   unvisitedQueue.push(make_pair (0, startV));
   
   while(!unvisitedQueue.empty())
   {
      long long currentV = unvisitedQueue.top().second;
      unvisitedQueue.pop();
      
      if (visited[currentV])
         continue;
         
      visited[currentV] = true;
      orderVisited.push_back(currentV);
      
      for (long long variable : G.neighbors(currentV))
      {
         if (!visited[variable])
         {
            double var;
            
         if (G.getWeight(currentV, variable, var))
         {
            double altDistance = distances[currentV] + var;
            
            if (altDistance < distances[variable])
            {
               distances[variable] = altDistance;
               pred[variable] = currentV;
               unvisitedQueue.push (make_pair(altDistance, variable));
            }
         }
         }
      }
   }

 return orderVisited;
}


//
// Using Dijkstra's, goes through the vertices to find the shortest possible path
//
void findShortestPath(long long startV, long long destiV, map<long long, double>& distances, map<long long, long long> preds)
{
  constexpr double INF = numeric_limits<double>::max();
  vector<long long> path;
  long long vtx = destiV;

  while(vtx != -1)
  {
    path.push_back(vtx);

    //updates vertex
    vtx = preds[vtx];

  }

  cout << "  Path: ";

  for (int i = (path.size() - 1); i >= 0; i--)
  {
    if (i == 0)
    {
      cout << path[i];
    }
    else
    {
      cout << path[i] << "->";
    }
  }
  cout << endl;
}


//
// Called after user inputs "@". Takes building and finds the closest footway ID,
// node ID, and distance with the help of navigationhelper.
// Prompts user to enter start and destination building names. Returns
// A warning if start building is not found
//
void navigate(Buildings& B,Nodes& nodes, Footways& Footways, graph G)
{

  string input1;
  string input2;
  Building startB(0,"","");
  Building destB(0,"","");
  double stored_distance;
  double lat1;
  double lon1;
  bool isEntrance;
  double d1;
  long long navihelp1;
  long long navihelp2;

  cout << "Enter start building name (partial or complete)>" << endl;

  getline(cin, input1);  

  for (Building& Build : B.MapBuildings)
  {
    if (Build.Name.find(input1) != string::npos) { // contains name:
      startB = Build;
      break;
    }
  }

  if (startB.ID == 0)
  {
    cout << "**Start building not found" << endl;
    return;
  }
  else
  {
    cout << "  Name: " << startB.Name << endl;

    pair<double, double> getLoc = startB.getLocation(nodes);

    cout << "  Approximate location: (" << getLoc.first << ", " << getLoc.second << ")" << endl;

    navihelp1 = navigationhelper(Footways, nodes, getLoc.first, getLoc.second);
    
  }

  cout << "Enter destination building name (partial or complete)>" << endl;
  getline(cin, input2);

  for (Building& Build: B.MapBuildings)
  {
    if (Build.Name.find(input2) != string::npos)
    {
      destB = Build;
      pair<double, double> getLoc2 = destB.getLocation(nodes);

      cout << "  Name: " << destB.Name << endl;
      cout << "  Approximate location: (" << getLoc2.first << ", " << getLoc2.second << ")" << endl;
      
      navihelp2 = navigationhelper(Footways,nodes, getLoc2.first, getLoc2.second);
      break;
    }
  }
  if (destB.ID == 0)
  {
    cout << "**Destination building not found" << endl;
    return;
  }

  map<long long, double> distances;
  map<long long, long long> preds;

  vector <long long> visited = Dijkstra(G, navihelp1, distances, preds);
  long long currNode = navihelp1;
  vector <long long> found_path;

  cout << "Shortest weighted path:" << endl;
  while(currNode != -1)
  {
    found_path.push_back(currNode);
    currNode = preds[currNode];
  }

  if(distances[navihelp2] == INF)
  {
    cout << "**Sorry, destination unreachable" << endl;
  }
  else{

    cout << "  # nodes visited: " << visited.size() << endl;
    cout << "  Distance: " << distances[navihelp2] << " miles" << endl;

    findShortestPath(navihelp1, navihelp2, distances, preds);
  }

}


//
// Checks for Sanity. (prints the weights in the graph)
//
void sanityCheck(graph& graph, Nodes& nodes, Footway& footways)
{
    for (size_t i = 0; i < footways.NodeIDs.size() - 1; i++)
    {
      long long from = footways.NodeIDs[i];
      long long to = footways.NodeIDs[i+1];

      double weight_from_to;
      double weight_to_from;

      bool found_f = graph.getWeight(from, to, weight_from_to);
      bool found_to = graph.getWeight(from, to, weight_to_from);

      cout << "Edge: (" << from << ", " << to << ", " << weight_from_to << ")" << endl;
      cout << "Edge: (" << to << ", " << from << ", " << weight_to_from << ")" << endl;
    }
}


//
// Checks for proper footways ID, if found then returns sanity check
//
void checkFootway(graph& graph, Nodes& nodes, Footways& footways)
{
  for (Footway fw : footways.MapFootways)
  {
    if (fw.ID == 986532630)
    {
      sanityCheck(graph,nodes, fw);
    }
  }
}

  


int main()
{
  cout << setprecision(12);
  XMLDocument xmldoc;
  Nodes nodes;
  Buildings buildings;
  Footways footways;
  graph G;
  cout << "** NU open street map **" << endl;

  string filename;

  cout << endl;
  cout << "Enter map filename> " << endl;
  getline(cin, filename);

  //
  // 1. load XML-based map file 
  //
  if (!osmLoadMapFile(filename, xmldoc))
  {
    // failed, error message already output
    return 0;
  }
  
  //
  // 2. read the nodes, which are the various known positions on the map:
  //
  nodes.readMapNodes(xmldoc);

  //
  // NOTE: let's sort so we can use binary search when we need 
  // to lookup nodes.
  //
  nodes.sortByID();

  //
  // 3. read the university buildings:
  //
  buildings.readMapBuildings(xmldoc);

  //
  // 4. read the footways, which are the walking paths:
  //
  footways.readMapFootways(xmldoc);

  graphBuilder(G, nodes, footways);

  //
  // 5. stats
  //
  cout << "# of nodes: " << nodes.getNumMapNodes() << endl;
  cout << "# of buildings: " << buildings.getNumMapBuildings() << endl;
  cout << "# of footways: " << footways.getNumMapFootways() << endl;
  cout << "# of graph vertices: " << G.NumVertices() << endl;
  cout << "# of graph edges: " << G.NumEdges() << endl;
  

  //
  // now let the user for search for 1 or more buildings:
  //
  while (true)
  {
    string name;

    cout << endl;
    cout << "Enter building name, * to list, @ to navigate, or $ to end> " << endl;

    getline(cin, name);

    if (name == "$") {
      break;
    }
    else if (name == "*") {
      buildings.print();
    }
    else if (name == "!"){
      checkFootway(G, nodes, footways);
    }
    else if (name == "@"){
      navigate(buildings, nodes, footways, G);
    }
    else {
      buildings.findAndPrint(name, nodes, footways);
    }

  }//while

  //
  // done:
  //
  cout << endl;
  cout << "** Done  **" << endl;
  cout << "# of calls to getID(): " << Node::getCallsToGetID() << endl;
  cout << "# of Nodes created: " << Node::getCreated() << endl;
  cout << "# of Nodes copied: " << Node::getCopied() << endl;
  cout << endl;

  return 0;
}
