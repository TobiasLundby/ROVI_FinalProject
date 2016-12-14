#include "SamplePlugin.hpp"
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

SamplePlugin::SamplePlugin(): RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png")){
	setupUi(this);
  //Robotics
	_timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));


	// now connect stuff from the ui component
	connect(_btnStart    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btnStop    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
  connect(_btnRestart    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btnNextFrame    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

	// Spin and sliders
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT( btnPressed()) );
  connect(_slider  ,SIGNAL(valueChanged(int)), this, SLOT( btnPressed()) );
	connect(_dtslider  ,SIGNAL(valueChanged(int)), this, SLOT( sliderDt(int)) );

  // Dropdowns
  connect(_drop_sequenceselect  ,SIGNAL(currentIndexChanged(QString )), this, SLOT(dropSequenceChanged(QString)) );
  connect(_drop_markerselect  ,SIGNAL(currentIndexChanged(QString )), this, SLOT(dropMarkerChanged(QString)) );
	connect(_spinBoxBackground  ,SIGNAL(currentIndexChanged(QString )), this, SLOT(dropBackgroundchanged(QString)) );


	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;

}

void SamplePlugin::sliderDt(int dt){
	this->dt = dt;
}


void SamplePlugin::dropBackgroundchanged(QString val){
	auto value = val.toUtf8().constData();

	std::string marker = background_path + value;
	std::cout << "Marker changed to: " << marker << "\n";

	Image::Ptr image = ImageLoader::Factory::load(marker);
	_bgRender->setImage(*image);

	getRobWorkStudio()->updateAndRepaint();
}
void SamplePlugin::dropMarkerChanged(QString val){
  auto value = val.toUtf8().constData();
  currentMarker = value;
  std::string marker = marker_path + value;
  std::cout << "Marker changed to: " << marker << "\n";

  std::cout << "debug: -" << value << "-" << std::endl;
	if(strcmp ("Markerpose",value) == 0){
		use_vision = false;
		std::cout << "Returning " << std::endl;
		return ; // Don't load marker if we use markerpose
	} else {
		use_vision = true;
	}

  // Set a new texture (one pixel = 1 mm)
  Image::Ptr imageMarker = ImageLoader::Factory::load(marker);
  _textureRender->setImage(*imageMarker);
  getRobWorkStudio()->updateAndRepaint();

   // Vision
   Mat image((*imageMarker).getHeight(),(*imageMarker).getWidth(), CV_8UC3);
   image.data = (uchar*)(*imageMarker).getImageData();

   cvtColor(image, image, CV_BGR2GRAY);

   if(image.empty()){
     std::cout << "Can not open image" << marker << std::endl;
   }
  siftdetector = new SIFTDetector(image);
	marker1detector = new ColorDetector();
}

void SamplePlugin::dropSequenceChanged(QString val){
  auto value = val.toUtf8().constData();
  std::string path = "/home/exchizz/SDU/Skole/7.Semester/ROVI/SamplePluginPA10/motions/";


  std::ifstream infile(path + value);
  if (infile.is_open()) {
    std::cout << "Motion file open" << path + value << std::endl;
  } else {
    std::cout << "motion file NOT open" << path + value  << std::endl;
  }
	double x, y, z, r, p, yaw;


  std::string line;
  motionVector.clear(); // Removing existing items in vector

  while (std::getline(infile, line)){
    std::istringstream iss(line);
    iss >> x >> y >> z >> r >> p >> yaw;

    motionVector.push_back( Pose(x,y,z,r,p,yaw) );
  }
  _slider->setMaximum(motionVector.size() -1);
  _spinBox->setMaximum(motionVector.size() - 1);
}
SamplePlugin::~SamplePlugin(){
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/exchizz/SDU/Skole/7.Semester/ROVI/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
	im = imread("/home/exchizz/SDU/Skole/7.Semester/ROVI/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin


  // Load default marker, background and sequence
	dropBackgroundchanged("color1.ppm");
  dropMarkerChanged("Markerpose");
  dropSequenceChanged("MarkerMotionSlow.txt");


}

void SamplePlugin::open(WorkCell* workcell){
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();

    _device = _wc->findDevice(_device_name);
    if(_device == NULL) {
        RW_THROW("Device " << _device_name << " was not found!");
    }

	log().info() << workcell->getFilename() << "\n";

	if (_wc != NULL) {
		// Add the texture render to this workcell if there is a frame for texture
		Frame* textureFrame = _wc->findFrame("MarkerTexture");
		if (textureFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
		}
		// Add the background render to this workcell if there is a frame for texture
		Frame* bgFrame = _wc->findFrame("Background");
		if (bgFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
		}

		// Create a GLFrameGrabber if there is a camera frame with a Camera property set
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		if (cameraFrame != NULL) {
			if (cameraFrame->getPropertyMap().has("Camera")) {
				// Read the dimensions and field of view
				double fovy;
				int width,height;
				std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
				std::istringstream iss (camParam, std::istringstream::in);
				iss >> fovy >> width >> height;
				// Create a frame grabber
				_framegrabber = new GLFrameGrabber(width,height,fovy);
				SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
				_framegrabber->init(gldrawer);
			}
		}
	}
}

void SamplePlugin::close() {
	log().info() << "CLOSE" << "\n";

	// Stop the timer
	_timer->stop();
	// Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed() {
	QObject *obj = sender();
  if(obj == _slider){
    i = _slider->value() - 1;
  } else if(obj==_btnRestart){
    log().info() << "Restarting sequence\n";
		i = 0;

		MovableFrame* _MarkerFrame = (MovableFrame*) _wc->findFrame("Marker");
		rw::math::Vector3D<> tempPos(motionVector[i].x, motionVector[i].y, motionVector[i].z);
		rw::math::RPY<> tempRot(motionVector[i].r, motionVector[i].p, motionVector[i].yaw);
		rw::math::Transform3D<double> MarkerTransform3D(tempPos, tempRot.toRotation3D());
		_MarkerFrame ->setTransform(MarkerTransform3D, _state);

		rw::math::Q q_initial(7,0,-0.65,0,1.80,0,0.42,0);
		_device->setQ(q_initial, _state);
		getRobWorkStudio()->setState(_state);
  } else if(obj==_btnStart){
    log().info() << "Starting timer\n";
    if (!_timer->isActive()){
		    _timer->start(100); // run 10 Hz
        //Image::Ptr image = ImageLoader::Factory::load("/home/exchizz/SDU/Skole/7.Semester/ROVI/SamplePluginPA10/backgrounds/color3.ppm");
        //_bgRender->setImage(*image);
    }
	} else if(obj==_btnStop){
		log().info() << "Stopping timer\n";
		// Toggle the timer on and off
		if (_timer->isActive()){
			_timer->stop();
    }
	} else if(obj==_spinBox){
		log().info() << "Jumping to frame: " << _spinBox->value() << "\n";
    i = _spinBox->value() -1; // -1 since next frame update will then be the frame we want to see
	} else if(obj == _btnNextFrame){
		std::cout << "Next frame" << std::endl;
		i++;
		timer();
	}
}

rw::math::VelocityScrew6D<double> SamplePlugin::calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired) {
    // Calculate dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * baseTtool.R().inverse());

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

rw::math::Q SamplePlugin::algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool,
                       const rw::math::Transform3D<double> baseTtool_desired, const rw::math::Q q_in) {
    auto baseTtool = device->baseTframe(tool, state);
    auto deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    rw::math::Q q = q_in;
    const double epsilon = 0.0001;
    while(deltaU.norm2() > epsilon) {
        auto J = device->baseJframe(tool, state);
        rw::math::Q deltaQ(J.e().inverse() * deltaU.e());
        q += deltaQ;
        device->setQ(q, state);
        baseTtool = device->baseTframe(tool, state);
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    }
    return q;
}


Jacobian SamplePlugin::GenerateImageJ(float u, float v){
	Jacobian Jimage(2,6);
	Jimage(0,0) = -f/z;
	Jimage(0,1) = 0;
	Jimage(0,2) = u/z;
	Jimage(0,3) = (u*v)/f;
	Jimage(0,4) = -(f*f+u*u)/f;
	Jimage(0,5) = v;

	// Second row
	Jimage(1,0) = 0;
	Jimage(1,1) = -f/z;
	Jimage(1,2) = v/z;
	Jimage(1,3) = (f*f+v*v)/f;
	Jimage(1,4) = (-u*v)/f;
	Jimage(1,5) = -u;

	return Jimage;
}


Q SamplePlugin::VelocityLimitReached(Q dq, float dt){
	auto maxQvel = _device->getVelocityLimits();
	auto cur_velocity = dq/dt;
	auto old_dq = dq;

	std::cout << "cur_velocity " << cur_velocity << std::endl;
	std::cout << "max_velocity " << maxQvel << std::endl;

	for(int i = 0; i < 7; i++){

		if( abs(dq[i]/dt) >= abs(maxQvel[i]) ){
				std::cout << "Hit vel. limit #" << i  << std::endl;
				dq[i] = (dq[i]/abs(dq[i]))*maxQvel[i]*dt;
		}
	}

	std::cout << "old dq " << old_dq << std::endl;
	std::cout << "new dq " << dq << std::endl;

	return dq;
}
void SamplePlugin::timer() {
  if(i == motionVector.size()-1){
    std::cout << "Reached end of sequence" << std::endl;
    _timer->stop();
		return;
  }

  // NOTE updates marker position with respect to base
  MovableFrame* _MarkerFrame = (MovableFrame*) _wc->findFrame("Marker");
  rw::math::Vector3D<> tempPos(motionVector[i].x, motionVector[i].y, motionVector[i].z);
  rw::math::RPY<> tempRot(motionVector[i].r, motionVector[i].p, motionVector[i].yaw);
  rw::math::Transform3D<double> MarkerTransform3D(tempPos, tempRot.toRotation3D());
  _MarkerFrame ->setTransform(MarkerTransform3D, _state);
	i++;

	if (_framegrabber != NULL) {


		if(firstrun){
			firstrun = false;
			rw::math::Q q_initial(7,0,-0.65,0,1.80,0,0.42,0);
			_device->setQ(q_initial, _state);
			getRobWorkStudio()->setState(_state);
		}
		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);
		std::vector<Point2f> interest_points;
		if(use_vision){
				Point2f center_point;
				std::cout << "i: " << i << std::endl;

				if (currentMarker == "Marker3.ppm"){
		    	Mat imgtmp;
		    	cvtColor(imflip, imgtmp, CV_BGR2GRAY);
		    	auto corners = siftdetector->GetCornersOfMarkerInScene(imgtmp);

					interest_points.push_back(corners[0]);
					interest_points.push_back(corners[3]);
					interest_points.push_back(corners[1]);
					interest_points.push_back(corners[2]);

		    	if(!lineIntersection(corners[0], corners[2], corners[1], corners[3], center_point)){
		      	std::cout << "Could not find SIFT intersection" << std::endl;
		    	}
		    	circle(imflip, center_point, 30, Scalar( 0, 255, 0), 10);
				} else if (currentMarker == "Marker1.ppm"){
					//std::cout << "not implemented yet" << std::endl;
					Mat imflip_bgr;
					cvtColor(imflip, imflip_bgr, COLOR_BGR2RGB);

					auto interest_points1 = marker1detector->FindMarker(imflip_bgr);
					for(auto elm: interest_points1){
						interest_points.push_back(elm);
					}

					if (interest_points.size()) {
		      	if(!lineIntersection(interest_points[0], interest_points[1], interest_points[3], interest_points[2], center_point)){
		        	std::cout << "Could not find SIFT intersection" << std::endl;
		      	}
		      	circle(imflip, center_point, 5, Scalar( 255, 0, 0), -1);
						//circle(imflip, interest_points.at(0), 5, Scalar( 127, 127, 127), -1); // Down left
						//circle(imflip, interest_points.at(1), 5, Scalar( 127, 127, 127), -1); // Up right
						//circle(imflip, interest_points.at(2), 5, Scalar( 127, 127, 127), -1); // Down right
						//circle(imflip, interest_points.at(3), 5, Scalar( 127, 127, 127), -1); // Up left
					} else {
						std::cout << "Marker1.ppm, didn't find 4 circles" << std::endl;
						return;
					}

				} else {
					imwrite("failed_detecion.png",imflip );
				}
			}


				// Offsets
				rw::math::Vector3D<> Offset0;
				rw::math::Vector3D<> Offset1;
				rw::math::Vector3D<> Offset2;

				if(!use_vision || currentMarker == "Marker3.ppm"){
					// Points to be used without vision
					Offset0[0] = 0.125;
					Offset0[1] = -0.125;
					Offset0[2] = 0;

					//rw::math::Vector3D<> Offset1(0.125,0.125,0);
					Offset1[0] = 0.125;
					Offset1[1] = 0.125;
					Offset1[2] = 0;

					//rw::math::Vector3D<> Offset2(-0.125,-0.125,0);
					Offset2[0] = -0.125;
					Offset2[1] = -0.125;
					Offset2[2] = 0;
				}

				if(use_vision && currentMarker == "Marker1.ppm"){
					std::cout << "Using Marker1" << std::endl;
				  // Points to be used with vision(Marker3)
					// rw::math::Vector3D<> Offset0(0.0525,0.0525,0);
					Offset0[0] = 0.0525;
					Offset0[1] = 0.0525;
					Offset0[2] = 0;
					// rw::math::Vector3D<> Offset1(-0.0525,-0.0525,0);
					Offset1[0] = -0.0525;
					Offset1[1] = -0.0525;
					Offset1[2] = 0;
					// rw::math::Vector3D<> Offset2(-0.0525,0.0525,0);
					Offset2[0] = -0.0525;
					Offset2[1] = 0.0525;
					Offset2[2] = 0;
				}
				// Given by assignment
				//rw::math::Vector3D<> Offset0(0,0,0);
				//rw::math::Vector3D<> Offset1(0.1,0,0);
				//rw::math::Vector3D<> Offset2(0,0.1,0);

				rw::math::Transform3D<double> Offset0T(Offset0);
				rw::math::Transform3D<double> Offset1T(Offset1);
				rw::math::Transform3D<double> Offset2T(Offset2);


				MovableFrame* _WorldFrame = (MovableFrame*) _wc->findFrame("WORLD");
				MovableFrame* _CameraFrame = (MovableFrame*) _wc->findFrame("Camera");
				MovableFrame* _MarkerFrame = (MovableFrame*) _wc->findFrame("Marker");


				auto MarkerTWorld = Kinematics::frameTframe(_MarkerFrame, _WorldFrame,_state);
				auto CameraTMarker = Kinematics::frameTframe(_CameraFrame, _MarkerFrame,_state);

				auto MarkerPosition0InMarkerFrame = Offset0T * MarkerTWorld*MarkerTransform3D.P();
				auto MarkerPosition1InMarkerFrame = Offset1T * MarkerTWorld*MarkerTransform3D.P();
				auto MarkerPosition2InMarkerFrame = Offset2T * MarkerTWorld*MarkerTransform3D.P();

				auto MarkerPosition0InCameraFrame = CameraTMarker*MarkerPosition0InMarkerFrame;
				auto MarkerPosition1InCameraFrame = CameraTMarker*MarkerPosition1InMarkerFrame;
				auto MarkerPosition2InCameraFrame = CameraTMarker*MarkerPosition2InMarkerFrame;

				int offset_x = 1024/2;
				int offset_y = 768/2;

				//z = MarkerPosition0InCameraFrame[2];

				//std::cout << "z: " << z << std::endl;

				double x0 =  MarkerPosition0InCameraFrame[0];
				double y0 =  MarkerPosition0InCameraFrame[1];
				double x1 =  MarkerPosition1InCameraFrame[0];
				double y1 =  MarkerPosition1InCameraFrame[1];
				double x2 =  MarkerPosition2InCameraFrame[0];
				double y2 =  MarkerPosition2InCameraFrame[1];

				double u0,v0,u1,v1,u2,v2;
				if(use_vision){
					// Track using vision
					u0 = interest_points[0].x - offset_x;
					v0 = interest_points[0].y - offset_y;
					u1 = interest_points[1].x - offset_x;
					v1 = interest_points[1].y - offset_y;
					u2 = interest_points[2].x - offset_x;
					v2 = interest_points[2].y - offset_y;
				} else {
					// Track using markers pose
					u0 = f*x0/z;
					v0 = f*y0/z;
					u1 = f*x1/z;
					v1 = f*y1/z;
					u2 = f*x2/z;
					v2 = f*y2/z;
				}

				auto Offset0InImageu = f*(-Offset0(0))/z;
				auto Offset0InImagev = f*Offset0(1)/z;
				auto Offset1InImageu = f*(-Offset1(0))/z;
				auto Offset1InImagev = f*Offset1(1)/z;
				auto Offset2InImageu = f*(-Offset2(0))/z;
				auto Offset2InImagev = f*Offset2(1)/z;

				Eigen::VectorXd dudv(6);
				dudv(0) = Offset0InImageu - u0;
				dudv(1) = Offset0InImagev - v0;
				dudv(2) = Offset1InImageu - u1;
				dudv(3) = Offset1InImagev - v1;
				dudv(4) = Offset2InImageu - u2;
				dudv(5) = Offset2InImagev - v2;

				circle(imflip, Point2f(offset_x+Offset0InImageu,offset_y+Offset0InImagev), 30, Scalar( 127, 0, 127), 2);
				circle(imflip, Point2f(offset_x+u0,offset_y+v0), 7, Scalar( 127, 0, 127), 2);
				circle(imflip, Point2f(offset_x+Offset1InImageu,offset_y+Offset1InImagev), 30, Scalar( 255, 0, 127), 2);
				circle(imflip, Point2f(offset_x+u1,offset_y+v1), 7, Scalar( 255, 0, 127), 2);
				circle(imflip, Point2f(offset_x+Offset2InImageu,offset_y+Offset2InImagev), 30, Scalar( 127, 127, 127), 2);
				circle(imflip, Point2f(offset_x+u2,offset_y+v2), 7, Scalar( 127, 127, 127), 2);
				//circle(imflip, Point2f(offset_x+u1,offset_y+v1), 30, Scalar( 255, 0, 127), 2);
				//circle(imflip, Point2f(offset_x+u2,offset_y+v2), 30, Scalar( 0, 127, 127), 2);

				//circle(imflip, Point2f(u1+offset_x,v1+offset_y), 30, Scalar( 127, 127, 127), 10);
				//circle(imflip, Point2f(u2+offset_x,v2+offset_y), 30, Scalar( 127, 127, 127), 10);

				auto Jimage0 = GenerateImageJ(u0,v0).e();
				auto Jimage1 = GenerateImageJ(u1,v1).e();
				auto Jimage2 = GenerateImageJ(u2,v2).e();

				Eigen::MatrixXd JCombined(6,6);

				JCombined.row(0) << Jimage0.row(0);
				JCombined.row(1) << Jimage0.row(1);

				JCombined.row(2) << Jimage1.row(0);
				JCombined.row(3) << Jimage1.row(1);

				JCombined.row(4) << Jimage2.row(0);
				JCombined.row(5) << Jimage2.row(1);


				// Eigen::MatrixXd U;
				// Eigen::VectorXd Sigma;
				// Eigen::MatrixXd V;
				// rw::math::LinearAlgebra::svd(JCombined,U, Sigma, V);
				// std::cout << " Sigma: " << Sigma << std::endl;


				// Create Jacobian
				auto Jimage = Jacobian(JCombined);

				// Gets transform from base to tool
				auto TToolWorld = _device->baseTframe(_CameraFrame, _state);

				/* Calculate S*/
				auto RBaseTool = TToolWorld.R().inverse();
				auto S = Jacobian(RBaseTool);

				/* Robot Jacobian */
				auto J = _device->baseJframe(_CameraFrame, _state); // Returns jacobian from tool to base frame.

				auto Zimage = (Jimage*S*J).e();
				auto ZimageT = Zimage.transpose();
				auto Zimage_tmp = ZimageT * (Zimage*ZimageT).inverse();
				auto dq = Zimage_tmp*dudv;

				// Get current robot configuration
				auto q_cur = _device->getQ(_state);

				// Set max vel
				Q dq_constrained = VelocityLimitReached(Q(dq), float(dt)/1000);

				// Add the change in robot configuration
				q_cur += Q(dq_constrained);



				// Update
        _device->setQ(q_cur, _state);
        getRobWorkStudio()->setState(_state);

				// Show in QLabel
				QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
				QPixmap p = QPixmap::fromImage(img);
				unsigned int maxW = 400;
				unsigned int maxH = 800;
				_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
	}
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

Q_EXPORT_PLUGIN(SamplePlugin);
