#ifndef _AIS_SCANMATCHER_HXX_
#define _AIS_SCANMATCHER_HXX_

#include <math_stuff/logutils.hh>
#include <vector>



  /** struct for returning a matching hypothese.
      The higher the score the better the match;
   */
struct MatcherResult{
  DTransformation2 pose;
  DSMatrix3 covariance;
  virtual double score()  const;
  virtual ~MatcherResult();
};

struct Scanmatcher{
  
  virtual ~Scanmatcher();
  
  //clears the local map
  virtual void clear();
  
  //integrates a scan at position robotPose in the local map
  //changeReference: if true, the center of the local map isset to robotPose. 
  virtual void integrateScan(const RobotLaser& laser, const DPose2& robotPose,  bool changeReference=false)=0;
  
  //global matching
  virtual void scanMatch(const RobotLaser& scan);
  
  //matching with an initial guess
  virtual void scanMatch(const RobotLaser& scan, const DPose2& initialGuess);

  //matching with an initial guess and a covariance
  virtual void scanMatch(const RobotLaser& scan, const DPose2& initialGuess, const DSMatrix3& covariance);

  //sparsifies the solutions
  void pruneResults(double linDist, double angDist);

  const std::vector<MatcherResult*>& results() const;

  
protected:
  void clearResults();
  virtual void sortResults();
  std::vector<MatcherResult*> _results;
};

#endif
