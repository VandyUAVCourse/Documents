#include "scanmatcher.hh"

#include <algorithm>

  using namespace std;

  double MatcherResult::score() const { return 0;}
  MatcherResult::~MatcherResult(){}

  
  Scanmatcher::~Scanmatcher(){
    clear();
    clearResults();
  };

  void Scanmatcher::clear(){}

  const std::vector<MatcherResult*>& Scanmatcher::results() const {return _results;}

  void Scanmatcher::scanMatch(const RobotLaser& scan){
    clearResults();
  }
  
  void Scanmatcher::scanMatch(const RobotLaser& scan, const DPose2& initialGuess){
    clearResults();
  }

  void Scanmatcher::scanMatch(const RobotLaser& scan, const DPose2& initialGuess, const DSMatrix3& covariance){
    clearResults();
  }
  
  void Scanmatcher::clearResults(){
    for (std::vector<MatcherResult*>::iterator it=_results.begin(); it!=_results.end(); it++){
      delete *it;
    }
    _results.clear();
  }

  struct MatcherResultPComparator{
    bool operator()(const MatcherResult* a, MatcherResult* b){
      return a->score()>b->score();
    }
  };

  void Scanmatcher::sortResults(){
    MatcherResultPComparator compare;
    std::sort(_results.begin(), _results.end(), compare);
  }

  void Scanmatcher::pruneResults(double linDist, double angDist){
     std::vector<MatcherResult*> pruned;

    int s=_results.size();
    if (!s)
      return;

    double sqLinDist=linDist*linDist;
    double sqRotDist=angDist*angDist;
    for (int i=0; i<s; i++){
      MatcherResult* r=_results[i];
      DTransformation2 pInv=r->pose.inv();
      int s2=pruned.size();
      bool found=false;
      for (int j=0; j<s2; j++){
	MatcherResult* r1=pruned[j];
	DTransformation2 p1=r1->pose;
	DTransformation2 pDelta=pInv*p1;
	DVector2 deltaT=pDelta.translation();
	double deltaR=pDelta.rotation();

	if (deltaT*deltaT<sqLinDist && deltaR*deltaR<sqRotDist){
	  found=true;
	  continue;
	} 
      }
      if (! found){
	pruned.push_back(r);
      } else {
	delete _results[i];
	_results[i]=0;
      }
    }
    _results=pruned;
  }

