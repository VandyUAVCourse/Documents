#ifndef __CMATCHER_HH__
#define __CMATCHER_HH__

#include <vector>
#include <math_stuff/gridmap.hh>
#include <math_stuff/logutils.hh>
#include <math_stuff/transformation2.hh>
#include <math_stuff/gridlinetraversal.hh>
#include <matcher/scanmatcher.hh>



  struct CMatcherResult: public MatcherResult{
    friend struct CMatcher;
    virtual double score() const;
  protected:
    double _score;
  };
  
  struct CMatcher: public Scanmatcher {
    CMatcher(double gridResolution=0.05, double angularResolution=(M_PI/720), double fillingDistance=0.2, double minScore=0.4, double workspaceRadius=100);
    virtual void clear();
    virtual void integrateScan(const RobotLaser& scan, const DPose2& robotPose, bool changeReference=false);
    virtual void scanMatch(const RobotLaser& scan, const DPose2& initialGuess);
    virtual void scanMatch(const RobotLaser& scan, const DPose2& initialGuess, const DSMatrix3& covariance);

    // parameters
    double gridResolution;
    double fillingDistance;
    double angularResolution;
    double minScore;
    double workspaceRadius;
    double matchingCroppingDistance;
    double matchingSparsificationDistance;
    double integrationCroppingDistance;
    

	 void integrateScan(std::vector<DVector2>& points, DPose2 laserPose, bool changeReference);
	 void scanMatch(std::vector<DVector2>& points, DPose2& laserPose, double xradius, double yradius, double thetaRadius);
	 
  protected:
    typedef GridMap<int> IntGridMap;
    IntGridMap workspace;
    std::vector<int*> markedCells; 
    std::vector<IVector2> markedIndices;

    IntGridMap convolvedWorkspace;
    std::vector<int*> convolvedMarkedCells; 
    

    //matcher specific functions
    
    void convolve();
  };

#endif
