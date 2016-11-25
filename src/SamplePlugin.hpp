#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"

#include <opencv2/opencv.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
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

	void stateChangedListener(const rw::kinematics::State& state);

private:
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

	QTimer* _timer;

	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;

	const std::string _device_name = "PA10";
	rw::models::Device::Ptr _device;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
