#ifndef _GRAPH_HH_
#define _GRAPH_HH_

#include <list>
#include <vector>
#include <map>
#include <set>
#include <assert.h>


template <typename GenericType>
struct GenericKeyComparator{
  inline bool operator () (const GenericType& a, const GenericType&b){
    return a<b;
  }
};

/**
Key:                   copy constructible, assignable
VertexInfo:            copy constructible, assignable
EdgeInfo:              copy constructible, assignable
KeyCompare:            StrictOrdering
Action:                function object which defines
                       bool operator()(Vertex* current, Vertex* parent, Vertex* parentEdge);
StoppingCriterion:     function object
                       bool operator()(Vertex* current);
Heuristic:             function object used to describe the heuristic in A*
                       double operator()(Vertex* current, Vertex* goal);
*/

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare= GenericKeyComparator<Key> >
struct Graph{
  struct Vertex;
  struct Edge;

  typedef std::map<Key, Vertex*, KeyCompare> VertexMap;
  typedef std::map<Edge*, Edge*>             EdgeMap;
  typedef std::map<Vertex*, Edge*>           VertexEdgeMap;

  struct Vertex: public VertexInfo{
    friend struct Graph;
    Vertex(const VertexInfo& info);
    Vertex& operator=(const VertexInfo& info);
    
    VertexEdgeMap leavingEdges;   //edges which lead to a node
    VertexEdgeMap leadingEdges;   //edges which enter into the node

    Key key;

    inline double&  distance() {return _distance;}
    inline Vertex*& parent() {return _parent;}
    inline Edge*& parentEdge() {return _parentEdge;}
    inline bool& mark() {return _mark;}
    
  protected:    
    bool    _mark;
    Vertex* _parent;
    Edge*   _parentEdge;
    double  _distance;
    double  _tempDistance;
  };

  struct Edge: public EdgeInfo{
    friend struct Graph;
    Edge(const EdgeInfo& info);
    Edge& operator=(const EdgeInfo& info);
    inline bool& mark() {return _mark;}
    Vertex* v1, *v2;
  protected:
    bool  _mark;
  };

  VertexMap vertices;
  EdgeMap edges;

  Graph();
  virtual ~Graph();
  Graph* clone();

  Edge* getEdge(Vertex* v1, Vertex* v2);
  Vertex* getVertex(Key& k);

  Edge* addEdge(Vertex*v1, Vertex* v2, EdgeInfo& info);
  Vertex* addVertex(Key& k, VertexInfo& info);

  bool removeEdge(Edge* e);
  bool removeVertex(Vertex* v);

  template <typename Action>
  void breadthFirstVisit(Vertex* v, Action& a, bool directed=true, bool clearMarks=true);

  template <typename Action>
  void depthFirstVisit(Vertex* v, Action& a, bool directed=true, bool clearMarks=true);

  template <typename StoppingCriterion>
  void dijkstra(Vertex* v, StoppingCriterion stop, bool directed=true);

  void dijkstra(Vertex* v, bool directed=true);

  template <typename Heuristic>
  bool astar(Vertex* start, Vertex* goal, Heuristic h, bool directed=true);

protected:
  template <typename Action>
  void _dfv(Vertex* v, Vertex* parent, Edge* edge, Action& a, bool& stop, bool directed);
  
};

#include "graph.hxx"


#endif
