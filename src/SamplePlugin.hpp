#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <rws/RobWorkStudio.hpp>
#include <QPushButton>
#include <fstream>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/rw.hpp>

//#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

//Vision
#include "functions.hpp"
#include "SIFTDetector.hpp"
#include "ColorDetector.hpp"

#include <rw/math/LinearAlgebra.hpp>


class Pose{
public:
		double x;
		double y;
		double z;
		double r;
		double p;
		double yaw;
	Pose(double x, double y, double z, double r, double p, double yaw){
		this->x = x;
		this->y = y;
		this->z = z;
		this->r = r;
		this->p = p;
		this->yaw = yaw;
	}
};

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:

	/* EDIT HERE */
	std::string plugin_path = "/home/exchizz/SDU/Skole/7.Semester/ROVI/SamplePluginPA10/";
	std::string log_path = "/home/exchizz/Dropbox/Mini-Project/Final_project/data/";
	int NumberOfPoints = 3; //{1,2,3}
  bool Testrun = false;
	/* END EDIT HERE */

  std::string marker_path = plugin_path + "markers/";
	std::string background_path = plugin_path + "backgrounds/";
	std::string sequence_path = plugin_path + "motions/";


	SamplePlugin();
	virtual ~SamplePlugin();
	virtual void open(rw::models::WorkCell* workcell);
	virtual void close();
	virtual void initialize();

private slots:
	void btnPressed();
	void timer();
	void dropSequenceChanged(QString value);
	void dropBackgroundchanged(QString val);
	void dropMarkerChanged(QString value);
	void sliderDt(int dt);
	void stateChangedListener(const rw::kinematics::State& state);

private:

	// Vision
	SIFTDetector * siftdetector;
	ColorDetector * marker1detector;
	bool use_vision = false;

	float f = 823;
	float z = 0.5;
	float dt = 1000;

	std::vector<Q> q_log;
	std::vector<rw::math::Transform3D<>> tool_log;
	std::vector<Point2f> error_log;

	// Robotics
	bool firstrun = true;
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
	int i = 0;
	std::string currentMarker = "Markerpose";
	std::string selectedSequence = "MarkerMotionSlow.txt";
	Jacobian GenerateImageJ(float u, float v);
	Q VelocityLimitReached(Q dq, float dt);

	// Log
	std::vector<Pose> motionVector;
	QTimer* _timer;

	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;

	const std::string _device_name = "PA10";
	rw::models::Device::Ptr _device;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
