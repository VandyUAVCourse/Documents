#include "laserparameters.hh"


LaserParameters::LaserParameters(int t, int nbeams, double _firstBeamAngle, double _angularStep, double _maxRange, double _accuracy, int _remissionMode){
  type=t;
  firstBeamAngle=_firstBeamAngle;
  angularStep =_angularStep;
  maxRange=_maxRange;
  laserPose=DTransformation2(0.,0.,0.);
  maxRange=maxRange;
  accuracy=_accuracy;
  remissionMode=_remissionMode;
  fov=angularStep*nbeams;
  if (nbeams>0){
    beams.resize(nbeams);
    double alpha=firstBeamAngle;
    for (int i=0; i<nbeams; i++){
      beams[i]=DTransformation2(0,0,alpha);
      alpha+=angularStep;
    }
  }
}

  LaserParameters::LaserParameters(int nbeams, double _firstBeamAngle, double _angularStep, double _maxRange){ 
    LaserParameters(0,nbeams, _firstBeamAngle, _angularStep, _maxRange, 0.1, 0);
}

