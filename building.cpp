/*building.cpp*/

//
// A building in the Open Street Map.
// 


#include <iostream>
#include <utility>

#include "building.h"

using namespace std;


//
// constructor
//
Building::Building(long long id, string name, string streetAddr)
  : ID(id), Name(name), StreetAddress(streetAddr)
{
  //
  // vector is default initialized by its constructor
  //
}

//
// containsThisNode
//
// Returns true if the building's nodes contains the given node id,
// false if not.
//

pair <double, double> Building::getLocation(Nodes& nodes)
{
  double avgLat;
  double avgLon;
  double totalLat = 0.0;
  double totalLon = 0.0;
  
  for (long long node_ids : this->NodeIDs)
  {
    double lat;
    double lon;
    bool isEntrance;
    nodes.find(node_ids, lat, lon, isEntrance);

    totalLat = totalLat + lat;
    totalLon = totalLon + lon;
  }

  avgLat = totalLat / this->NodeIDs.size();
  avgLon = totalLon / this->NodeIDs.size();

  return make_pair(avgLat,avgLon);

}


bool Building::containsThisNode(long long nodeid)
{
  for (long long id : this->NodeIDs)
  {
    if (nodeid == id)
      return true;
  }

  // 
  // if get here, not found:
  //
  return false;
}

//
// print
// 
// prints information about a building --- id, name, etc. -- to
// the console. The function is passed the Nodes for searching 
// purposes.
//
void Building::print(Nodes& nodes)
{
  cout << this->Name << endl;
  cout << "Address: " << this->StreetAddress << endl;
  cout << "Building ID: " << this->ID << endl;

  cout << "Approximate location: (" << this->getLocation(nodes).first << ", " << this->getLocation(nodes).second << ")" << endl;
  cout << "Nodes:" << endl;
  for (long long nodeid : this->NodeIDs)
  {
    cout << "  " << nodeid << ": ";

    double lat = 0.0;
    double lon = 0.0;
    bool entrance = false;

    bool found = nodes.find(nodeid, lat, lon, entrance);

    if (found) {
      cout << "(" << lat << ", " << lon << ")";

      if (entrance)
        cout << ", is entrance";

      cout << endl;
    }
    else {
      cout << "**NOT FOUND**" << endl;
    }
  }//for
}

//
// adds the given nodeid to the end of the vector.
//
void Building::add(long long nodeid)
{
  this->NodeIDs.push_back(nodeid);
}
