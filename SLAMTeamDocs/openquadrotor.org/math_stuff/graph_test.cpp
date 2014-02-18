#include "graph.hh"
#include <string>
#include <iostream>
#include <stdio.h>

using namespace std;

struct MyFancyVertex;
struct MyFancyEdge;
typedef Graph<int, MyFancyVertex, MyFancyEdge> MyFancyGraph;

struct MyFancyVertex{
  friend struct ParentAssigner;
  int level;
  std::string info;
};

struct MyFancyEdge{
  MyFancyGraph::Vertex* top;
  int length;
  std::string info;
  virtual double distance() {return 1;}
};


struct ParentAssigner{
  bool operator()(MyFancyGraph::Vertex* current, MyFancyGraph::Vertex* parent, MyFancyGraph::Edge* edge){
    current->parent()=parent;
    if (parent){
      current->level=parent->level+1;
      current->parentEdge()=edge;
    } else {
      current->level=0;
      current->parentEdge()=0;
    }
    return false;
  }
};

MyFancyGraph::Vertex*  topNode(MyFancyGraph::Edge* e, int* length=0){
  MyFancyGraph::Vertex* p1=e->v1;
  MyFancyGraph::Vertex* p2=e->v2;
  int l=0;
  while (p1 && p2 && p1!=p2){

    assert(p1);
    assert(p2);

    if (!p1->parent()){
      if (length)
	*length=l;
      return p1;
    }
    if (!p2->parent()){
      if (length)
	*length=l;
      return p2;
    }
    if (p1->level > p2->level){
      p1=p1->parent();
      l++;
    }else if (p1->level < p2->level){
      p2=p2->parent();
      l++;
    } else {
      p1=p1->parent();
      p2=p2->parent();
      l+=2;
    }
  }
  if (p1==p2){
    if (length)
      *length=l;
    return p1;
  }
  if (length)
    *length=-1;
  return 0;
}


int main(int argc, char** argv){
  cerr << "graph creation" << endl;
  MyFancyGraph g;
  std::cerr << "Adding Vertices" << endl;
  for (int i=0; i<100; i++){
    cerr << ".";
    char  buf[10];
    sprintf(buf, "%d", i);
    MyFancyVertex vinfo;
    vinfo.info=buf;
    g.addVertex(i,vinfo);
  }
  cerr << endl;
  cerr << "Adding Edges" << endl;
  for (int i=1; i<100; i++){
    for (int j=0; j<i; j++){
      if (i==j+1 || i==j+2 || i+j ==10 || i+j==11){
	cerr << ".";
	MyFancyGraph::Vertex* v1, *v2;
	v1=g.getVertex(i);
	v2=g.getVertex(j);
	char  buf[10];
	sprintf(buf, "%d", i);
	MyFancyEdge einfo;
	einfo.info=buf;
	g.addEdge(v2,v1,einfo);
      }
    }
    cerr << endl;
  }

  std::cerr << "cloning" << endl;
  MyFancyGraph* g2=g.clone();
  std::cerr << "done, vertices=" << g2->vertices.size() << " edges=" << g2->edges.size() << endl;

  std::cerr << "removing vertices" << endl;
  for (MyFancyGraph::VertexMap::iterator it=g2->vertices.begin(); it!=g2->vertices.end(); it++){
    if (!(it->first%2)){
      int key=it->first;
      MyFancyGraph::Vertex* v=g2->getVertex(key);
      it++; //avance the iterator (to prevent it to become invalid when changing the structure)
      g2->removeVertex(v);
    }
  }
  std::cerr << "done, vertices=" << g2->vertices.size() << " edges=" << g2->edges.size() << endl;

  
  cerr << "Breadth first visit" << endl;
  ParentAssigner parentAssigner;
  g2->breadthFirstVisit(g2->vertices.begin()->second, parentAssigner, false);
  cerr << "Done" << endl;
  
  cerr << "Dijkstra" << endl;
  g2->dijkstra(g2->vertices.begin()->second, false);
  cerr << "Done" << endl;

  for (MyFancyGraph::VertexMap::iterator it=g2->vertices.begin(); it!=g2->vertices.end(); it++){
    cerr << " id:" << it->second->key << " " << it->second->distance() << endl;
  }



  for (MyFancyGraph::EdgeMap::iterator it=g2->edges.begin(); it!=g2->edges.end(); it++){
    it->second->top=topNode(it->second, &(it->second->length));
    cerr << "t=" << it->second->top << endl;
    std::cerr << "(" << it->second->v1->key << "," 
	      << it->second->v2->key << ") parent:";
    if (it->second->top)
      cerr << it->second->top->key;
    else
      cerr << -1;
    cerr << " length=" << it->second->length;
    cerr<< endl;
  }

  delete g2;

}
