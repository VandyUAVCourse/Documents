#include "transformation2.hh"

std::vector<DVector2> sparsify(const std::vector<DVector2>& points, const double& sqDist){
	std::vector<DVector2> rp;
	if (! points.size())
		return rp;
	rp.reserve(points.size());
	DVector2 lastPoint=points[0];
	for (int i=0; i<(int)points.size(); i++){
		DVector2 delta=lastPoint-points[i];
		if (delta*delta>sqDist){
			lastPoint=points[i];
			rp.push_back(lastPoint);
		}
	}
	return rp;
}
