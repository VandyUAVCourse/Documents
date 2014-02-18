#include "histcmatcher.h"

HistCMatcher::HistCMatcher(int _bufferSize, double gridResolution, double angularResolution, double fillingDistance, double minScore, double workspaceRadius) : CMatcher( gridResolution, angularResolution, fillingDistance, minScore, workspaceRadius){
	bufferSize = _bufferSize;
	pointsBuffer.clear();
	poseBuffer.clear();
	referenceBuffer.clear();
}

void HistCMatcher::integrateScan(std::vector<DVector2>& points, DPose2 laserPose, bool changeReference){
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
	CMatcher::integrateScan(points, laserPose, changeReference);
}


void HistCMatcher::scanMatch(std::vector<DVector2>& points, DPose2& laserPose, double xradius, double yradius, double thetaRadius){
	if (bufferSize > 0){
		clear();
		std::list<std::vector<DVector2> >::iterator itL = pointsBuffer.begin();
		std::list<DPose2>::iterator itP = poseBuffer.begin();
		std::list<bool>::iterator itR = referenceBuffer.begin();
		int i = 0;
		for (itL = pointsBuffer.begin(); itL != pointsBuffer.end() ; itL++, itP++, itR++){
			CMatcher::integrateScan(*itL, *itP, i==0 || *itR);
			i++;
		}
	}
	CMatcher::scanMatch(points, laserPose, xradius, yradius, thetaRadius);
}
