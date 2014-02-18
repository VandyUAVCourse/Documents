#ifndef _MULTI_RESOLUTION_CMATCHER_H_
#define _MULTI_RESOLUTION_CMATCHER_H_

#include "cmatcher.hh"
#include <vector>

using namespace std;

class MultiResolutionCMatcher {
	public:
		MultiResolutionCMatcher (int numMatchers, double maxResolution, double worldsize);
		void scanMatch(std::vector<DVector2>& currentpoints, DPose2& laserPose,  double xradius, double yradius, double thetaRadius);
		const vector<MatcherResult*> results() { return matcher.at(numMatchers-1)->results();};
		
		void integrateScan(std::vector<DVector2>& currentpoints, DPose2& pose, bool moveOriginToThisScan);
	private:
		vector<CMatcher*> matcher;
		int numMatchers;
		vector<MatcherResult*> res;
		double maxRes;
};
#endif // _MULTI_RESOLUTION_CMATCHER_H_
