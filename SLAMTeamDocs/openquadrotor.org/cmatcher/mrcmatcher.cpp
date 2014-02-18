#include "mrcmatcher.h"

MultiResolutionCMatcher::MultiResolutionCMatcher(int _nM, double res, double ws){
	numMatchers = _nM;
	matcher.resize(_nM);
	double d = 1;
	maxRes = res;
	for (int i=0; i<_nM; i++){
		if (i!=0)
			d = d * 2;
		std::cerr << "resolution of matcher "<< i << " is " << res/d << std::endl;
		matcher.at(i) = new CMatcher(res/d, (M_PI/720), 0.2, 0.4, ws);
	}
}

void MultiResolutionCMatcher::scanMatch(std::vector<DVector2>& currentpoints,   DPose2& laserPose, double xradius, double yradius, double thetaRadius){
	double d = 1;
	for (int i=0; i<numMatchers; i++){
		if (i!=0)
			d = d * 2;
			
		matcher.at(i)->scanMatch(currentpoints, laserPose, xradius/d, yradius/d, thetaRadius/d);
		const vector<MatcherResult*> results = matcher[i]->results();
		if (!results.size()){
			std::cerr << "***** SCAN MATCHING FAILED! ***** "<< std::endl;
		} else {
			laserPose = results.at(0)->pose.toPoseType();
		}
		std::cout << laserPose.x() << " " << laserPose.y() << " ";
	}
	std::cout << std::endl;
}

void MultiResolutionCMatcher::integrateScan(std::vector<DVector2>& currentpoints, DPose2& pose, bool reset){
	for (int i=0; i<numMatchers; i++){
		matcher.at(i)->integrateScan(currentpoints, pose, reset);
	}
}
