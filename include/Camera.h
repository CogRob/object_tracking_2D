#pragma once

#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class CCamera
{
public:
  CCamera(std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height);
  CCamera(std::string &img_path, bool color, int imgIdx, std::string &intrinsic, std::string &distortion, std::string &imgext);
  ~CCamera(void);

  IplImage* getImage();
  inline CvMat* getIntrinsicParams()      { return intrinsic_;  }
  inline CvMat* getDistortionParams()     { return distortion_; }
  inline int getWidth()                   { return width_;      }
  inline int getHeight()                  { return height_;     }
  inline IplImage* getEdge()              { return img_edge_;   }
  inline bool IsColorImage()              { return color_;      }
  inline bool IsCamera()                  { return (cam_type_ == CAM_SEQ) ? (false) : (true); }

  enum {CAM_SEQ, CAM_NORMAL, CAM_FIREI, CAM_FLEA, CAM_OPENNI, CAM_ACH};

protected:
  CvMat *intrinsic_;
  CvMat *distortion_;
  std::string m_strImgPath;
  bool color_;
  int m_nImgIdx;
  int m_nStartIdx;
  int width_;
  int height_;
  CvCapture* capture_;
  IplImage *img_mapx_;
  IplImage *img_mapy_;
  IplImage *img_input_;
  IplImage *img_edge_;
  std::string img_ext_;
  bool verbose_;
  bool m_bFull;
  int cam_type_;

  cv::VideoCapture video_capture_;
};
