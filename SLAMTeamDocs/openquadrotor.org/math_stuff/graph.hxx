#include <assert.h>
#include <queue>
#include <values.h>


  using namespace std;

template <typename VertexP>
struct DistanceSelector{
  inline double operator()(const VertexP& v){
    return v->distance();
  }
};

template <typename VertexP>
struct DistanceHeuristicSelector{
  inline double operator()(const VertexP& v){
    return v->distance()+v->tempDistance();
  }
};

template <typename VertexP, typename DistanceFunction=DistanceSelector<VertexP> >
struct HeapVertexDistanceComparator{
  DistanceFunction distanceFunction;

  HeapVertexDistanceComparator(bool selectMin=false){
    _selectMin=selectMin;
  }

  inline bool operator()(VertexP v1, VertexP v2){
    assert(v1);
    assert(v2);
    if (_selectMin){
      return distanceFunction(v1)<distanceFunction(v2);
    }
    return distanceFunction(v1)>distanceFunction(v2);
  }
protected:
  bool _selectMin;
};


template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex& 
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex::operator=(const VertexInfo& info){
  VertexInfo* tmp = (VertexInfo*) this;
  *tmp=info;
  return *this;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex::Vertex(const VertexInfo & info){
  VertexInfo* tmp = (VertexInfo*) this;
  *tmp=info;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge::Edge(const EdgeInfo& info){
  EdgeInfo* tmp=(EdgeInfo*) this;
  *tmp=info;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge& 
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge::operator=(const EdgeInfo& info){
  EdgeInfo* tmp=(EdgeInfo*) this;
  *tmp=info;
  return *this;
}



template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Graph(){}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::~Graph(){
  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::const_iterator it=vertices.begin(); 
       it!=vertices.end(); it++)
    delete it->second;
  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::EdgeMap::const_iterator it=edges.begin(); 
       it!=edges.end(); it++)
    delete it->second;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>* Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::clone(){
  Graph<Key, VertexInfo, EdgeInfo, KeyCompare>* g=new Graph();

  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::const_iterator it=vertices.begin(); 
       it!=vertices.end(); it++){
    g->addVertex(it->second->key, *((VertexInfo*)(it->second)));
  }
  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::EdgeMap::const_iterator it=edges.begin(); 
       it!=edges.end(); it++){
    Vertex* v1=g->getVertex(it->second->v1->key);
    Vertex* v2=g->getVertex(it->second->v2->key);
    assert(v1);
    assert(v2);
    g->addEdge(v1,v2,*((EdgeInfo*)it->second));
  }
  return g;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge *  Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::getEdge(Vertex* v1, Vertex* v2){
  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=v1->leavingEdges.find(v2);
  if (it==v1->leavingEdges.end())
    return 0;
  return it->second;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* 
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::getVertex(Key& k){
  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::iterator it=vertices.find(k);
  if (it==vertices.end())
    return 0;
  return it->second;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::addVertex(Key& k, VertexInfo& info){
  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* v=getVertex(k);
  if (v)
    return 0;
  v = new typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex(info);
  v->_mark=false;
  v->key=k;
  vertices.insert(make_pair(k,v));
  return v;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*
Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::addEdge(Vertex*v1, Vertex* v2, EdgeInfo& info){
  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge* e=getEdge(v1,v2);
  if (e)
    return 0;
  e=new typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge(info);
  e->v1=v1;
  e->v2=v2;
  e->_mark=0;
  v1->leavingEdges.insert(make_pair(v2,e));
  v2->leadingEdges.insert(make_pair(v1,e));
  edges.insert(make_pair(e,e));
  return e;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
bool Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::removeEdge(Edge* e){
  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::EdgeMap::iterator it=edges.find(e);
  if (it==edges.end())
    return false;
  edges.erase(it);

  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator v_eit=e->v1->leavingEdges.find(e->v2);
  assert (v_eit!=e->v1->leavingEdges.end());
  e->v1->leavingEdges.erase(v_eit);

  v_eit=e->v2->leadingEdges.find(e->v1);
  assert (v_eit!=e->v2->leadingEdges.end());
  e->v2->leadingEdges.erase(v_eit);

  delete e;
  return true;
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare >
bool Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::removeVertex(Vertex* v){
  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::iterator it=vertices.find(v->key);
  if (it==vertices.end())
    return false;

  typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap edgesToRemove=v->leavingEdges;
  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator e_it=edgesToRemove.begin();
       e_it!=edgesToRemove.end(); e_it++){
    removeEdge(e_it->second);
  }

  edgesToRemove=v->leadingEdges;
  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator e_it=edgesToRemove.begin();
       e_it!=edgesToRemove.end(); e_it++){
    removeEdge(e_it->second);
  }
  
  vertices.erase(it);
  delete v;
  return true;
}




template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare>
template <typename Action>
void Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::breadthFirstVisit(Vertex* v, Action& a, bool directed, bool clearMarks){
  // clear all the _marks

  if (clearMarks){
    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::iterator it=vertices.begin(); 
	 it!=vertices.end(); it++)
      it->second->_mark=false;
  }

  typedef std::deque<typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* > VertexDeque;
  VertexDeque q;
  bool stop=a(v,0,0);
  if (! stop){
    q.push_back(v);
    v->_mark=true;
  }

  while (! q.empty()){
    typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* v=q.front();
    q.pop_front();

    //insert all the un_marked nodes which are reachable from v
    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=v->leavingEdges.begin(); it!=v->leavingEdges.end() ; it++){
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
      if (n->_mark)
	continue;
      stop=a(n,v,e);
      if (!stop){
	q.push_back(n);
	n->_mark=true;
      }
    }

    if (directed)
      continue;

    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=v->leadingEdges.begin(); it!=v->leadingEdges.end() && ! stop ; it++){
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
      if (n->_mark)
	continue;
      stop=a(n,v,e);
      if (!stop){
	q.push_back(n);
	n->_mark=true;
      }
    }


  }
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare>
template <typename Action>
void Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::depthFirstVisit(Vertex* v, Action& a, bool directed, bool clearMarks){

  if (clearMarks){
    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::iterator it=vertices.begin(); 
	 it!=vertices.end(); it++)
      it->second->_mark=false;
  }
  bool stop=false;
  _dfv(v,0,0,a,stop,directed);
}

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare>
template <typename Action>
void Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::_dfv(Vertex* v, Vertex* parent, Edge* edge, Action& a, bool& stop, bool directed){
  if (v->_mark)
    return;
  v->_mark=true;
  stop |= stop=a(v,parent,edge);
  if (stop)
    return;
  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=v->leavingEdges.begin(); it!=v->leavingEdges.end() && ! stop; it++){
    typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
    typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
    if (n->_mark)
      continue;
    _dfv(n,v,e,a,stop,directed);
  }
  if ( directed)
    return;
  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=v->leadingEdges.begin(); it!=v->leadingEdges.end() && ! stop; it++){
    typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
    typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
    if (n->_mark)
      continue;
    _dfv(n,v,e,a,stop,directed);
  }
}



template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare>
template <typename StoppingCriterion>
void Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::dijkstra(Vertex* v, StoppingCriterion stop, bool directed){
  using namespace std;
  typedef HeapVertexDistanceComparator<typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex*> DistanceCompare;
  typedef std::priority_queue<typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex*, 
    std::vector <typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex*>,
    DistanceCompare > PriorityQueue;

  DistanceCompare comp;
  PriorityQueue q(comp);

  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::iterator it=vertices.begin();
       it!=vertices.end(); it++){
    if (it->second!=v){
      it->second->_distance=MAXDOUBLE;
    } else {
      v->_distance=0;
    }
    it->second->_mark=false;
    it->second->_parent=0;
    it->second->_parentEdge=0;
  }

  q.push(v);

  while (!q.empty()){
    typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* current=q.top();
    if (stop(current))
      return;
    q.pop();
    if (current->_mark)
      continue;
    current->_mark=true;

    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=current->leavingEdges.begin(); it!=current->leavingEdges.end(); it++){
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
      double  newDistance=current->_distance+e->distance();
      if (newDistance<n->_distance){
	n->_distance=newDistance;
	n->_parent=current;
	n->_parentEdge=e;
	q.push(n);
      }
    }

    if (directed)
      continue;

    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=current->leadingEdges.begin(); it!=current->leadingEdges.end(); it++){
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
      double  newDistance=current->_distance+e->distance();
      if (newDistance<n->_distance){
	n->_distance=newDistance;
	n->_parent=current;
	n->_parentEdge=e;
	q.push(n);
      }

    }
  }
}

template <typename VertexP>
struct DummyStoppingCriterion{
  bool operator()(VertexP v){
    return false;
  }
};


template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare>
void Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::dijkstra(Vertex* v, bool directed){
  DummyStoppingCriterion< typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* > stop;
  dijkstra(v,stop,directed);
}

template <typename VertexP>
struct DummyHeuristic{
  double operator()(VertexP n, VertexP goal){
    return 0.;
  }
};

template <typename Key,  typename VertexInfo, typename EdgeInfo, typename KeyCompare>
template <typename Heuristic>
bool Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::astar(Vertex* start, Vertex* goal,  Heuristic h, bool directed){
  using namespace std;

  typedef HeapVertexDistanceComparator<typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex*, 
    DistanceHeuristicSelector<typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex*> > DistanceCompare;
  typedef std::priority_queue<typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex*, 
    std::vector <typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex*>,
    DistanceCompare > PriorityQueue;

  DistanceCompare comp;
  PriorityQueue q(comp);

  for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexMap::iterator it=vertices.begin();
       it!=vertices.end(); it++){
    if (it->second!=start){
      it->second->_distance=MAXDOUBLE;
    } else {
      start->_distance=0;
      start->tempDistance=h(start,goal);
    }
    it->second->_mark=false;
    it->second->_parent=0;
    it->second->_parentEdge=0;
  }

  q.push(start);

  while (!q.empty()){
    typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* current=q.top();
    if (current==goal){
      return true;
    }
    q.pop();
    if (current->_mark)
      continue;
    current->_mark=true;

    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=current->leavingEdges.begin(); it!=current->leavingEdges.end(); it++){
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
      double  newDistance=current->_distance+e->distance();
      if (newDistance<n->_distance){
	n->_distance=newDistance;
	n->_tempDistance=n->_distance+h(n,goal);
	n->_parent=current;
	n->_parentEdge=e;
	q.push(n);
      }
    }

    if (directed)
      continue;

    for (typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::VertexEdgeMap::iterator it=current->leadingEdges.begin(); it!=current->leadingEdges.end(); it++){
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Vertex* n=it->first;
      typename Graph<Key, VertexInfo, EdgeInfo, KeyCompare>::Edge*   e=it->second;
      double  newDistance=current->_distance+e->distance();
      if (newDistance<n->_distance){
	n->_distance=newDistance;
	n->_tempDistance=n->_distance+h(n,goal);
	n->_parent=current;
	n->_parentEdge=e;
	q.push(n);
      }
    }
  }
  return false;
}

