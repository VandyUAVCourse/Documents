/** This implements the SLAM algorithm  */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
//#include <octomap/math/Vector3.cpp>

using namespace octomap; 

int main(int argc, char** argv) {

	//create empty tree with resolution 0.1
	OcTree tree (0.1);

	point3d origin ( imux, imuy, imuz);

	insertScan(/* point cloud from Nolan */, 




}







