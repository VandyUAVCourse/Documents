#include "lasertransformator.h"
#include <ipcMessages/qc_laser_messages.h>
#include <ipcMessages/qc_imu_messages.h>
#include <ipcMessages/qc_odometry_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_imu_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <ipcInterfaces/qc_odometry_interface.h>
#include <QGLViewer/qglviewer.h>
#include <qapplication.h>
#include <iostream>
///max history number of 250!
qc_odometry_odometry_laserpoints_message olmsg;
qc_odometry_odometry_message omsg;
qc_odometry_velocity_message vmsg;
int msgIndex = 0;
uint max_history = 1000;
class WorldTest : public QGLViewer {
	public:
		WorldTest(){
			glLineWidth(2.0);
			glDisable(GL_LIGHTING);
			glPointSize(3.0);
			setGridIsDrawn();
			setSceneRadius(3000.);
			setBackgroundColor(QColor(100,100,100));
		}
		void draw () {
			///draw previous poses
			glPushAttrib(GL_ENABLE_BIT);
			glDisable(GL_LIGHTING);
			glPushMatrix();
			list<DVector3>::iterator it;
			list<vector<DVector2> >::iterator lit = map_points.begin();
			list<DVector3>::iterator oit = orientations.begin();
			double col = 0.;
			glPointSize(3.0);
			for (it = poses.begin(); it!=poses.end() && lit!=map_points.end() && oit!=orientations.end(); it++, lit++, oit++){
				glPushMatrix();
				col += 1./max_history;
				glColor3f(col,1.f,0.1f);
				DVector3& p = *it;
				vector<DVector2>& points = *lit;
				glTranslated(p.x(),p.y(),p.z());
				DVector3& op = *oit;
				glRotated(op.z()*180./M_PI,0,0,1);
				glBegin(GL_POINTS);
					glVertex3f(0,0,0);
				glEnd();
				glColor3f(1.0,0.1f,col);
				glBegin(GL_LINES);
					for (uint i=0; i<points.size(); i++){
						glVertex3f(points[i].x(), points[i].y(), 0);
						glVertex3f(points[i].x(), points[i].y(), 10);
					}
				glEnd();
				glPopMatrix();
			}
			glPopMatrix();
			glPopAttrib();
		}
		list<vector<DVector2> > map_points;
		list<DVector3> poses;
		list<DVector3> orientations;
	protected:
		void keyPressEvent(QKeyEvent* e){
			if (e->key() == Qt::Key_R){
				e->accept();
				map_points.clear();
				poses.clear();
				orientations.clear();
				return;
			} else {
				QGLViewer::keyPressEvent(e);
			}
			updateGL();
		}
};

WorldTest* viewer;

void qc_odometry_odometry_laserpoints_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &olmsg, sizeof(olmsg));
	vector<DVector2> p(olmsg.num_laser_readings);
	for (int i=0; i<olmsg.num_laser_readings; i++){
		p[i] = DVector2(olmsg.lx[i]*1e2, olmsg.ly[i]*1e2);
	}
	viewer->map_points.push_back(p);
	DVector3 pose(olmsg.x*1e2, olmsg.y*1e2, 0);
	viewer->poses.push_back(pose);
	viewer->orientations.push_back(Quaternion<double>(olmsg.q0,olmsg.q1,olmsg.q2,olmsg.q3).toAngles());
	if (viewer->map_points.size() > max_history){ ///keep history of 250
		viewer->map_points.pop_front();
		viewer->poses.pop_front();
		viewer->orientations.pop_front();
	}
	IPC_freeByteArray(callData);	
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&olmsg);
	viewer->updateGL();
}

void qc_odometry_odometry_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &omsg, sizeof(omsg));
	std::cerr << "ODOM[" <<  msgIndex << "]= " << (omsg.timestamp_sec + 1e-6*omsg.timestamp_usec) << " " << omsg.x << " " << omsg.y << " " << omsg.z << " ";
	DVector3 a = Quaternion<double> (omsg.q0, omsg.q1, omsg.q2, omsg.q3).toAngles();
	std::cerr << a.x()*180./M_PI << " " << a.y()*180./M_PI << " " << a.z()*180./M_PI << std::endl;
	msgIndex++;
	IPC_freeByteArray(callData);	
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&omsg);
}

void qc_odometry_velocity_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &vmsg, sizeof(vmsg));
	/*
	std::cerr << " VELOCITY= [" << vmsg.x << " " << vmsg.y << " " << vmsg.z << " " << vmsg.q0 << " " << vmsg.q1 << " " << vmsg.q2 << " " << vmsg.q3 << "] ";
	std::cerr << "{" << vmsg.vx << " " << vmsg.vy << " " << vmsg.vz << "} {";
	std::cerr << vmsg.avx << " " << vmsg.avy << " " << vmsg.avz << "} {";
	std::cerr << vmsg.aix << " " << vmsg.aiy << " " << vmsg.aiz << "} {";
	std::cerr << vmsg.axtanp << " " << vmsg.aytanr << "}" << std::endl;
	*/
	IPC_freeByteArray(callData);	
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&vmsg);
}




int main (int argc, char* argv[]){
	
	olmsg.num_z_readings = 0;
	olmsg.zr = 0;
	olmsg.num_laser_readings = 0;
	olmsg.lx = 0;
	olmsg.ly = 0;
	
	omsg.num_z_readings = 0;
	omsg.zr = 0;
	
	QApplication application(argc, argv);
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(-1);
	}
	WorldTest t;
	viewer = &t;
	
	qc_odometry_subscribe_odometry_laserpoints_message(qc_odometry_odometry_laserpoints_message_handler, 1 ,NULL);
	qc_odometry_subscribe_odometry_message(qc_odometry_odometry_message_handler, 1 ,NULL);
	qc_odometry_subscribe_velocity_message(qc_odometry_velocity_message_handler, 1, NULL);

	//application.setMainWidget(viewer);

	viewer->show();

	while (viewer->isShown()){
		application.processEvents();
		IPC_listen(1);
	}
	
	qc_odometry_unsubscribe_odometry_laserpoints_message(qc_odometry_odometry_laserpoints_message_handler);
	qc_odometry_unsubscribe_odometry_message(qc_odometry_odometry_message_handler);

	return 0;
	
	
	
}
