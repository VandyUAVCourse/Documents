#include "lasertransformator.h"
#include <ipcMessages/qc_laser_messages.h>
#include <ipcMessages/qc_imu_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_imu_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <configSender/ipcParamReader.h>
#include <QGLViewer/qglviewer.h>
#include <qapplication.h>

///from which index on are the beams reflected by the mirror?

int heightFirstBeam = 620;
int heightSkipBeamStart = 12;
int heightLastBeam = 682;
int heightSkipBeamStop = 5;


class WorldTest : public QGLViewer {
	public:
	WorldTest(){
		glLineWidth(2.0);
		glDisable(GL_LIGHTING);
		glPointSize(3.0);
		setGridIsDrawn();
		setSceneRadius(300.);
		setBackgroundColor(QColor(100,100,100));
	}
	void draw () {
		glPushAttrib(GL_ENABLE_BIT);
		glDisable(GL_LIGHTING);
		glPushMatrix();
		glTranslated(0,0,100); ///100cm in pos z
		glPointSize(3.0);
		///draw the 3d points, its 2d projection and the 3d points of the mirror
		glColor3f(0.,0.,1.); //3d laser blue
		glBegin(GL_LINES);
		for (uint i=0; i<laser3d.size(); i++){
			glVertex3f(0.,0.,0.);
			glVertex3f(laser3d[i].x()*1e2, laser3d[i].y()*1e2, laser3d[i].z()*1e2);
		}
		glEnd();
		glBegin(GL_POINTS);
		for (uint i=0; i<laser3d.size(); i++){
			glVertex3f(laser3d[i].x()*1e2, laser3d[i].y()*1e2, laser3d[i].z()*1e2);
		}
		glEnd();
		glColor3f(1.,0.,0.); // 3d projection in red
		glBegin(GL_POINTS);
		for (uint i=0; i<laser2d.size(); i++){
			glVertex3f(laser2d[i].x()*1e2, laser2d[i].y()*1e2, 0);
		}
		glEnd();
		glColor3f(0.,1.,0.); // mirror in green 
		glBegin(GL_POINTS);
		for (int i=0; i<(int)mirror3d_finalpoints.size(); i++){
			///skip beams in white
			if (i <= heightSkipBeamStart || i >= (int)mirror3d_finalpoints.size()-heightSkipBeamStop)
				glColor3f(1.,1.,1.);
			else
				glColor3f(0.,1.,0.);
			glVertex3f(mirror3d_finalpoints[i].x()*1e2, mirror3d_finalpoints[i].y()*1e2, mirror3d_finalpoints[i].z()*1e2);
		}
		glEnd();
		glBegin(GL_LINES);
		for (int i=0; i<(int)mirror3d.size(); i++){
			if (i <= heightSkipBeamStart || i >= (int) mirror3d_finalpoints.size()-heightSkipBeamStop)
				glColor3f(1.,1.,1.);
			else
				glColor3f(0.,1.,0.);
			glVertex3f(0,0,0);
			glVertex3f(mirror3d[i].x()*1e2, mirror3d[i].y()*1e2, mirror3d[i].z()*1e2);
			glVertex3f(mirror3d[i].x()*1e2, mirror3d[i].y()*1e2, mirror3d[i].z()*1e2);
			glVertex3f(mirror3d_finalpoints[i].x()*1e2, mirror3d_finalpoints[i].y()*1e2, mirror3d_finalpoints[i].z()*1e2);
		}
		glEnd();
		
		glPopMatrix();
		glPopAttrib();
	}
	vector<DVector3> laser3d;
	vector<DVector2> laser2d;
	vector<DVector3> mirror3d;
	vector<DVector3> mirror3d_finalpoints;
};

WorldTest* viewer;
qc_imu_imu_message imsg;
qc_laser_laser_message lmsg;
bool imu_init = false;
LaserTransformator transformator;
vector<double> ranges;
vector<double> angles;


void qc_imu_imu_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &imsg, sizeof(imsg));
	IPC_freeByteArray(callData);
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	imu_init = true;
	
}

void qc_laser_laser1_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &lmsg, sizeof(lmsg));
	IPC_freeByteArray(callData);
	///calculate ranges and angles of laser
	ranges.resize(lmsg.num_ranges);
	angles.resize(lmsg.num_ranges);
	int count = 0;
	double alpha = lmsg.startAngle - lmsg.incrementAngle;
	for (int i=0; i<lmsg.num_ranges && i < heightFirstBeam; i++){
		alpha += lmsg.incrementAngle;
		if (lmsg.ranges[i] < 20)
			continue;
		ranges[count] = lmsg.ranges[i] * 1e-3;
		angles[count] = alpha;
		count++;
	}
	ranges.resize(count);
	angles.resize(count);
	if (!imu_init){
		///identity quaternion
		imsg.q0 = 1;
		imsg.q1 = 0;
		imsg.q2 = 0;
		imsg.q3 = 0;
	}
	Quaternion<double> qimu(imsg.q0, imsg.q1, imsg.q2, imsg.q3);
	transformator.calculate3dLaserPoints(ranges, angles, viewer->laser3d, qimu, false);
	transformator.calculate2dLaserPoints(ranges, angles, viewer->laser2d, qimu, false);
	ranges.resize(lmsg.num_ranges);
	angles.resize(lmsg.num_ranges);
	count = 0;
	alpha = lmsg.startAngle + (heightFirstBeam-1) * lmsg.incrementAngle;
	for (int i=heightFirstBeam; i<lmsg.num_ranges; i++){
		alpha += lmsg.incrementAngle;
		if (lmsg.ranges[i] < 20)
			continue;
		ranges[count] = lmsg.ranges[i] * 1e-3;
		angles[count] = alpha;
		count++;
	}
	ranges.resize(count);
	angles.resize(count);
	transformator.calculate3dMirrorPoints(ranges, angles, viewer->mirror3d_finalpoints, qimu, 0.065, viewer->mirror3d, false);
	///
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&lmsg);
	viewer->updateGL();
}





///test with 3d gui
int main (int argc, char* argv[]){
	QApplication application(argc,argv);
	lmsg.ranges = 0;
	lmsg.num_ranges = 0;
	if (qc_ipc_connect(argv[0]) <= 0){
			return 0;
	}
	
	
	///---->retrieve parameters
	IPCParamReader paramReader;
	if (!paramReader.getIntValue("laser_mirror_parameters", 4, &heightFirstBeam, &heightSkipBeamStart, &heightLastBeam, &heightSkipBeamStop))
		return 0;
	///<---retrieve parameters
	
	
  	WorldTest t;
	viewer = &t;
	
	qc_imu_subscribe_imu_message (qc_imu_imu_message_handler, 10, NULL);
	qc_laser_subscribe_laser1_message (qc_laser_laser1_message_handler, 10, NULL);
   application.setMainWidget(viewer);

	viewer->show();

	while (viewer->isShown()){
		application.processEvents();
		IPC_listen(1);
	}

   return 0;

}

