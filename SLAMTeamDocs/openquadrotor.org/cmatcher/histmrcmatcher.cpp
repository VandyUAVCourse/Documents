#include "histmrcmatcher.h"

HistMRCMatcher::HistMRCMatcher(int _bufferSize, int _numMatchers, double gridResolution, double angularResolution, double fillingDistance, double minScore, double workspaceRadius){
	 
	bufferSize = _bufferSize;
	numMatchers = _numMatchers;
	maxResolution = gridResolution;
	pointsBuffer.clear();
	poseBuffer.clear();
	referenceBuffer.clear();
	double d = 1;
	matchers.resize(numMatchers);
	for (int i=0; i<numMatchers; i++){
		if (i!=0)
			d = d * 2;
		matchers[i] = new CMatcher(gridResolution/d, angularResolution, fillingDistance, minScore, workspaceRadius);
		std::cerr << "Matcher " << i << " running at " << gridResolution/d << " m resolution" << std::endl;
	}
	
}

HistMRCMatcher::~HistMRCMatcher(){
	for (int i=0; i<numMatchers; i++)
	delete matchers[i];
}

void HistMRCMatcher::integrateScan(std::vector<DVector2>& points, DPose2 laserPose, bool changeReference){
	if (bufferSize > 0){ 
		pointsBuffer.push_back(points);
		poseBuffer.push_back(laserPose);
		referenceBuffer.push_back(changeReference);
		if ((int)pointsBuffer.size() > bufferSize){
			pointsBuffer.pop_front();
			poseBuffer.pop_front();
			referenceBuffer.pop_front();
		}
	}
	for (int i=0; i<numMatchers; i++)
		matchers[i]->integrateScan(points, laserPose, changeReference);
}


void HistMRCMatcher::scanMatch(std::vector<DVector2>& points, DPose2& laserPose, double xradius, double yradius, double thetaRadius){
	if (bufferSize > 0){
		for (int i=0; i<numMatchers; i++)
			matchers[i]->clear();
		std::list<std::vector<DVector2> >::iterator itL = pointsBuffer.begin();
		std::list<DPose2>::iterator itP = poseBuffer.begin();
		std::list<bool>::iterator itR = referenceBuffer.begin();
		int cr = 0;
		for (itL = pointsBuffer.begin(); itL != pointsBuffer.end() ; itL++, itP++, itR++){
			for (int i=0; i<numMatchers; i++){
				matchers[i]->integrateScan(*itL, *itP, cr==0 || *itR);
			}
			cr++;
		}
	}
	double d = 1;
	double prevRes = maxResolution;
	double dx = xradius;
	double dy = yradius;
	double dth = thetaRadius;
	for (int i=0; i<numMatchers; i++){
		if (i!=0){
			d = d * 2;
			dx = 1.5 * prevRes;
			dy = 1.5 * prevRes;
			dth = 1.5 * prevRes;
			prevRes = prevRes / d;
		}
		//matchers[i]->scanMatch(points, laserPose, xradius/d, yradius/d, thetaRadius/d);
		matchers[i]->scanMatch(points, laserPose, dx, dy, dth);
		const std::vector<MatcherResult*> results = matchers[i]->results();
		if (results.size() > 0)
			laserPose = results[0]->pose.toPoseType();
		//std::cout << laserPose.x() << " " << laserPose.y() << " ";
	}
	//std::cout << std::endl;
}

const std::vector<MatcherResult*> HistMRCMatcher::results(int i){
	if (i < 0 || i >= numMatchers){
		i = 0;
	}
	return matchers[i]->results();
}
