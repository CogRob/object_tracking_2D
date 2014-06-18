#include "Camera.h"
#include <iomanip> 

CCamera::CCamera(std::string &img_path, bool color, int imgIdx, std::string &intrinsic, std::string &distortion, std::string &imgext)
  : verbose_(false)
  , cam_type_(CAM_SEQ)
{
  // load saved images from disk
  m_strImgPath  = img_path;
  color_        = color;
  m_nImgIdx     = imgIdx;
  intrinsic_    = (CvMat*)cvLoad(intrinsic.c_str());
  distortion_   = (CvMat*)cvLoad(distortion.c_str());
  img_ext_      = imgext;

  // Check the image resolution and save it
  std::stringstream ss;
  ss << m_strImgPath << "/" << "img" << std::setw(4) << std::setfill('0') << m_nImgIdx << "." << img_ext_;
  IplImage* image = cvLoadImage(ss.str().c_str(), color_ ? CV_LOAD_IMAGE_COLOR : CV_LOAD_IMAGE_GRAYSCALE);
  width_ = image->width;
  height_ = image->height;
  std::cout << "image resolution: (" << width_ << ", " << height_ << ")" << std::endl;

  img_mapx_ = NULL;
  img_mapy_ = NULL;
  img_input_ = NULL;
}

CCamera::CCamera(std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height)
{
  // capture images from camera
  if(cam_name.compare("normal") == 0)
    cam_type_ = CAM_NORMAL;
  else if(cam_name.compare("fire-i") == 0)
    cam_type_ = CAM_FIREI;
  else if(cam_name.compare("flea") == 0)
    cam_type_ = CAM_FLEA;
  else if(cam_name.compare("openni") == 0)
    cam_type_ = CAM_OPENNI;
  else if(cam_name.compare("ach") == 0)
    cam_type_ = CAM_ACH;
  else
  {
    std::cerr << "Unknown camera type. Force to use normal camera type" << std::endl;
    cam_type_ = CAM_NORMAL;
  }
    
  intrinsic_    = (CvMat*)cvLoad(intrinsic.c_str());
  distortion_   = (CvMat*)cvLoad(distortion.c_str());

  if(cam_type_ == CAM_OPENNI)
  {
    video_capture_.open(CV_CAP_OPENNI);
    if(!video_capture_.isOpened())
    {
      std::cerr << "Openni sensor cannot be openned" << std::endl;
      return;
    }

    video_capture_.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ);
    if(video_capture_.get(CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT))
    {
      int w = static_cast<int>(video_capture_.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_WIDTH));
      int h = static_cast<int>(video_capture_.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_HEIGHT));
      assert(w == width);
      assert(h == height);
    }
    else
    {
      std::cerr << "Device doesn't contain image generator." << std::endl;
      return;
    }
  }
  else if (cam_type_ != CAM_ACH)
  {
    capture_ = cvCaptureFromCAM(0);
    assert(capture_);

    double fps = cvGetCaptureProperty(capture_, CV_CAP_PROP_FPS);
    std::cout << "Camera FPS: " << (int)fps << std::endl;

    double msec = (double)cvGetCaptureProperty(capture_, CV_CAP_PROP_FOURCC);

    // Force to (width x height)
    cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_WIDTH, width);
    cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_HEIGHT, height);

    CvSize size = cvSize(
      (int)cvGetCaptureProperty(capture_, CV_CAP_PROP_FRAME_WIDTH),
      (int)cvGetCaptureProperty(capture_, CV_CAP_PROP_FRAME_HEIGHT)
      );

    assert(size.width == width);
    assert(size.height == height);
  }

  CvSize size = cvSize(width, height);
  // create the undistort map images
  img_mapx_ = cvCreateImage(size, IPL_DEPTH_32F, 1);
  img_mapy_ = cvCreateImage(size, IPL_DEPTH_32F, 1);
  // innit the undistort map
  cvInitUndistortMap(intrinsic_, distortion_, img_mapx_, img_mapy_);
  // create the main image
  img_input_ = cvCreateImage(size, IPL_DEPTH_8U, 3);
  img_edge_ = NULL;

  width_ = width;
  height_ = height;
  std::cout << "image resolution: (" << width_ << ", " << height_ << ")" << std::endl;

  color_ = true;
}

CCamera::~CCamera(void)
{
  if(cam_type_ != CAM_SEQ && cam_type_ != CAM_OPENNI)
    cvReleaseCapture(&capture_);

  if(img_mapx_)      cvReleaseImage(&img_mapx_);
  if(img_mapy_)      cvReleaseImage(&img_mapy_);
  if(img_input_)     cvReleaseImage(&img_input_);
}

IplImage* CCamera::getImage()
{
  if(cam_type_ != CAM_SEQ)
  {
    if(cam_type_ == CAM_OPENNI)
    {
      cv::Mat bgrImage;

      if(!video_capture_.grab())
      {
        std::cerr << "Can not grab images." << std::endl;
        return NULL;
      }

      video_capture_.retrieve(bgrImage, CV_CAP_OPENNI_BGR_IMAGE);
      IplImage* frame = new IplImage(bgrImage);
      cvCopy(frame, img_input_);
    }
    else
    {
			// grab image from camera
			int res=cvGrabFrame (capture_);
			IplImage* frame = cvRetrieveFrame(capture_);

      assert(frame);
      cvCopy(frame, img_input_);
    }
		// check whether its a flea cam
    if(cam_type_ == CAM_FLEA)
      cvFlip(img_input_); // input from flea camera is upside down

    IplImage* t = cvCloneImage(img_input_);
    cvRemap(t, img_input_, img_mapx_, img_mapy_); // rectify
    cvReleaseImage(&t);
    return img_input_;
  }
  else // image sequence
  {
    std::stringstream ss;
    ss << m_strImgPath << "/" << "img" << std::setw(4) << std::setfill('0') << m_nImgIdx << "." << img_ext_;
    if(verbose_) std::cout << "Load image: " << ss.str() << std::endl;
    // Read an image from the saved image sequence
    IplImage* image = cvLoadImage(ss.str().c_str(), color_ ? CV_LOAD_IMAGE_COLOR : CV_LOAD_IMAGE_GRAYSCALE);

    // Read edge image if there are edge images in the same folder. (e.g. Berkeley edges, scale space edges)
    // If there are no edge images, 'img_edge_' would be NULL
    ss.str(std::string());
    ss << m_strImgPath << "/" << "img" << std::setw(4) << std::setfill('0') << m_nImgIdx << "bedges." << "png";
    img_edge_ = cvLoadImage(ss.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    // Increase the image index
    m_nImgIdx++;
    return image;
  }
}
