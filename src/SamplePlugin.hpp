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

  std::string marker_path = "/home/exchizz/SDU/Skole/7.Semester/ROVI/SamplePluginPA10/markers/";


	//Robotics
	SamplePlugin();
	virtual ~SamplePlugin();

	virtual void open(rw::models::WorkCell* workcell);

	virtual void close();

	virtual void initialize();

	rw::math::VelocityScrew6D<double> calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired);
	rw::math::Q algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool, const rw::math::Transform3D<double> baseTtool_desired, const rw::math::Q q_in);
private slots:
	void btnPressed();
	void timer();

	void dropSequenceChanged(QString value);
	void dropMarkerChanged(QString value);

	void stateChangedListener(const rw::kinematics::State& state);

private:


  Jacobian GenerateImageJ(float u, float v);
	// Vision
	SIFTDetector * siftdetector;
	ColorDetector * marker1detector;

	float f = 823;
	float z = 0.5;

	// Robotics
	bool firstrun = true;



	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
	int i = 0;
	std::string currentMarker;

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
