#ifndef _HIST_MULTI_RESOLUTION_CMATCHER
#define _HIST_MULTI_RESOLUTION_CMATCHER

#include "cmatcher.hh"
#include <list>
#include <vector>
#include <iostream>

///History-CMatcher
struct HistMRCMatcher {
	///if buffersize <= 0, its behaviour is like cmatcher,
	///otherwise it is working on the last buffersize laser measurements
	public:
		HistMRCMatcher(int bufferSize = -1, int numMatchers = 1, double gridResolution=0.05, double angularResolution=(M_PI/720), double fillingDistance=0.2, double minScore=0.4, double workspaceRadius=20);
                ~HistMRCMatcher();
	
		void scanMatch(std::vector<DVector2>& points, DPose2& laserPose, double xradius, double yradius, double thetaRadius);
		void integrateScan(std::vector<DVector2>& points, DPose2 laserPose, bool changeReference);
		const std::vector<MatcherResult*> results(int i);
		const std::vector<MatcherResult*> results() {return results(0);};
	private:
		std::vector<CMatcher*> matchers;
		std::list<std::vector<DVector2> > pointsBuffer;
		std::list<DPose2> poseBuffer;
		std::list<bool> referenceBuffer;
		int bufferSize;
		int numMatchers;
		double maxResolution;
};


#endif // _HIST_MULTI_RESOLUTION_CMATCHER
