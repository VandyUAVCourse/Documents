#ifndef _HIST_C_MATCHER_H_
#define _HIST_C_MATCHER_H_

#include "cmatcher.hh"
#include <list>

///History-CMatcher
struct HistCMatcher : public CMatcher {
	///if buffersize <= 0, its behaviour is like cmatcher,
	///otherwise it is working on the last buffersize laser measurements
	public:
	HistCMatcher(int bufferSize = -1, double gridResolution=0.05, double angularResolution=(M_PI/720), double fillingDistance=0.2, double minScore=0.4, double workspaceRadius=100);
	
	void scanMatch(std::vector<DVector2>& points, DPose2& laserPose, double xradius, double yradius, double thetaRadius);
	void integrateScan(std::vector<DVector2>& points, DPose2 laserPose, bool changeReference);
	
	private:
	std::list<std::vector<DVector2> > pointsBuffer;
	std::list<DPose2> poseBuffer;
	std::list<bool> referenceBuffer;
	int bufferSize;
};
#endif // _HIST_C_MATCHER_H_
