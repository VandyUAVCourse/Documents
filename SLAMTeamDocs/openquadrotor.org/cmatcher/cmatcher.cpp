#include "cmatcher.hh"
#include <assert.h>
#include <algorithm>

#include <values.h>

  double CMatcherResult::score() const{
    return _score;
  }

//   static int kernel3x3[3][3]={{1, 5,   1},
// 			      {5, 100, 5},
// 			      {1, 5,   1}};

  static int kernel3x3[3][3]={{0, 1, 0},
			      {1, 4, 1},
			      {0, 1, 0}};

  CMatcher::CMatcher(double _gridResolution, 
			     double _angularResolution, 
			     double _fillingDistance, 
			     double _minScore, 
			     double _workspaceRadius){
    
    gridResolution=_gridResolution;
    workspaceRadius=_workspaceRadius;
    angularResolution=_angularResolution;
    fillingDistance=_fillingDistance;
    minScore=_minScore;
    int cells=(int) (2*workspaceRadius/gridResolution);
    workspace         =IntGridMap(IVector2(cells, cells), gridResolution, DVector2(-workspaceRadius, -workspaceRadius), int(0));
    convolvedWorkspace=IntGridMap(IVector2(cells, cells), gridResolution, DVector2(-workspaceRadius, -workspaceRadius), int(0));
    markedCells.reserve(cells*cells);
    convolvedMarkedCells.reserve(cells*cells);
    markedIndices.reserve(cells*cells);
    
    matchingSparsificationDistance=0.05;
    matchingCroppingDistance=1e3;
    integrationCroppingDistance=1e3;
  }

  void CMatcher::integrateScan(const RobotLaser& scan, const DPose2& robotPose, bool changeReference){
    RobotLaser laser(scan);
    laser.crop(integrationCroppingDistance);
    std::vector<DVector2> points=laser.cartesian();
    DTransformation2 currentLaserPoseOnRobot(laser.laserParams.laserPose);
    DTransformation2 currentPose(robotPose);
    currentPose=currentPose*currentLaserPoseOnRobot;
    DPose2 cp2=currentPose.toPoseType();
    integrateScan(points,cp2,changeReference);
  }

  void CMatcher::scanMatch(const RobotLaser& scan, const DPose2& initialGuess){
    RobotLaser laser(scan);
    laser.crop(matchingCroppingDistance);
    std::vector<DVector2> points=laser.cartesian();
    std::vector<DVector2> sparsified=sparsify(points, matchingSparsificationDistance*matchingSparsificationDistance);
    DTransformation2 currentLaserPoseOnRobot(laser.laserParams.laserPose);
    DTransformation2 currentPose(initialGuess);
    currentPose=currentPose*currentLaserPoseOnRobot;
    DPose2 cp2=currentPose.toPoseType();
    scanMatch(sparsified, cp2, 0.2, 0.2, 0.2);
    
    // translate the laser pose to the robot pose
    DTransformation2 clpinv=currentLaserPoseOnRobot.inv();
    for (int i=0; i<(int)_results.size(); i++){
      DTransformation2 p=_results[i]->pose;
      p=p*clpinv;
      _results[i]->pose=p.toPoseType();
    }
  }

  void CMatcher::scanMatch(const RobotLaser& scan, const DPose2& initialGuess, const DSMatrix3& covariance){
    RobotLaser laser(scan);
    laser.crop(matchingCroppingDistance);
    std::vector<DVector2> points=laser.cartesian();
    std::vector<DVector2> sparsified=sparsify(points, matchingSparsificationDistance*matchingSparsificationDistance);
    DTransformation2 currentLaserPoseOnRobot(laser.laserParams.laserPose);
    DTransformation2 currentPose(initialGuess);
    currentPose=currentPose*currentLaserPoseOnRobot;
    DPose2 cp2=currentPose.toPoseType();
    // HACK
    double xx=3*sqrt(covariance.values[0][0]);
    double yy=3*sqrt(covariance.values[1][1]);
    double tt=3*sqrt(covariance.values[2][2]);
    scanMatch(sparsified, cp2, xx,yy,tt);

    DTransformation2 clpinv=currentLaserPoseOnRobot.inv();
    // translate the laser pose to the robot pose
    for (int i=0; i<(int) _results.size(); i++){
      DTransformation2 p=_results[i]->pose;
      p=p*clpinv;
      _results[i]->pose=p.toPoseType();
    }
  }


  void CMatcher::integrateScan(std::vector<DVector2>& points, DPose2 laserPose, bool changeReference){
    clearResults();
    // compute the offset of the workspace so that the center of the robot will be in the middle of the workspace
    double xoff=laserPose.x()-workspaceRadius;
    double yoff=laserPose.y()-workspaceRadius;
    if (changeReference){
      workspace.offset=DVector2(xoff, yoff);
      convolvedWorkspace.offset=DVector2(xoff, yoff);
    }
    GridLineTraversalLine line;

    DVector2 previous(MAXDOUBLE,MAXDOUBLE);
    IVector2 iprevious(0.,0.);

    int s=points.size();
    DTransformation2 t(laserPose);

    for (int i=0; i<s; i++){
      DVector2 p=t*points[i];
      IVector2 ip=workspace.world2map(p);
      DVector2 delta=previous-p;
      if (delta*delta<fillingDistance*fillingDistance){
	GridLineTraversal::gridLine(iprevious, ip, &line);
	for (int j=0; j<line.num_points; j++){
	  IVector2 cp=line.points[j];
	  assert(workspace.isInside(cp));
	  int& c=workspace.cell(cp);
	  if (!c){
	    c=1;
	    markedCells.push_back(&c);
	    markedIndices.push_back(ip);
	  }
	}
      } else {
	  assert(workspace.isInside(ip));
	  int& c=workspace.cell(ip);
	  if (!c){
	    c=1;
	    markedCells.push_back(&c);
	    markedIndices.push_back(ip);
	  }
      }
      previous=p;
      iprevious=ip;
    }
  }

  void CMatcher::clear(){
    int s=markedCells.size();
    for (int i=0; i<s; i++){
      *(markedCells[i])=0;
    }
    markedCells.clear();
    markedIndices.clear();
  }

  void CMatcher::convolve(){
    int s=convolvedMarkedCells.size();
    for (int i=0; i<s; i++){
      *(convolvedMarkedCells[i])=0;
    }
    convolvedMarkedCells.clear();

    for (int i=0; i<(int)markedIndices.size(); i++){
      IVector2 v=markedIndices[i];
      for (int y=0; y<3; y++)
	for (int x=0; x<3; x++){
	  IVector2 v2=v;
	  v2.x()+=x-1;
	  v2.y()+=y-1;
	  if (! convolvedWorkspace.isInside(v2))
	    continue;
	  int* c=&convolvedWorkspace.cell(v2);
	  bool wasZero=(!*c);
	  (*c) += kernel3x3[x][y];
	  bool isNotZero=(bool)(*c);
	  if (wasZero && isNotZero)
	    convolvedMarkedCells.push_back(c);
	}
    }

  }

  void CMatcher::scanMatch(std::vector<DVector2>& points, DPose2& laserPose, double xradius, double yradius, double thetaRadius){
    using namespace std;    
    clearResults();
    convolve();
    int kArea=0;
    for (int i=0; i<3; i++)
      for (int j=0; j<3; j++)
	kArea+=kernel3x3[i][j];
    
    std::vector<DVector2> dpoints;
    dpoints.resize(points.size());
    std::vector<IVector2> ipoints;
    ipoints.resize(points.size());
    int ixradius=(int) (xradius/gridResolution);
    int iyradius=(int) (yradius/gridResolution);

    for (double theta=laserPose.theta()-thetaRadius; theta<laserPose.theta()+thetaRadius; theta+=angularResolution){
 
      DTransformation2 t(laserPose);
      t.setRotation(theta);
      //translate all the points according to the laser guess;
      int s=points.size();
      for(int i=0; i<s; i++){
	dpoints[i]=t*points[i];
	ipoints[i]=convolvedWorkspace.world2map(dpoints[i]);
      }
      //now do an extensive matching for every x and every y in the confidence region
      for (int yy=-iyradius; yy<iyradius; yy++){
	for (int xx=-ixradius; xx<ixradius; xx++){
	  int score=0;
	  for (int i=0; i<s; i++){
	    IVector2 k(ipoints[i].x()+xx, ipoints[i].y()+yy);
	    if (! convolvedWorkspace.isInside(k)){
	      continue;
	    }
	    int c=convolvedWorkspace.cell(k);
	    score+=c;
	  }
	  DPose2 currentPose(laserPose.x()+xx*gridResolution, laserPose.y()+yy*gridResolution, theta);
	  //std::cerr << "cp: " << currentPose.x() << " " << currentPose.y() << " " << currentPose.theta() << std::endl;
	  if ((score) > minScore*points.size()*kArea){
	    CMatcherResult* result = new CMatcherResult;
	    result->pose=currentPose;
	    result->_score=score/kArea;
	    _results.push_back(result);
	  }
	}
      }
    }
    if (_results.size()){
      sortResults();
    }
  }

