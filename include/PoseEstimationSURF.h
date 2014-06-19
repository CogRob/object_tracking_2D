#pragma once

// To do:
// . create a pose init base and divide surf-based and line-based

#include <iostream>
#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "Image/Image.h"
#include "Image/ImageIO.h"
#include "Fitline/LFLineFitter.h"
#include "Fdcm/LMLineMatcher.h"

using namespace std;

class CObjectModel;
class CPoseEstimationSURF
{
public:
  CPoseEstimationSURF(int width, int height, std::string& img_path, CObjectModel* obj_model, CvMat* intrinsic_params, CvMat* distortion_params, std::string& objName, bool dispaly=true);
  ~CPoseEstimationSURF(void);
  CvMat* estimatePose(int &num_of_corr);
  void PF_estimatePoses(int &num_of_corr, int numOfParticle, vector<CvMat*>& states);
  int PF_estimatePosesFDCM(float maxThreshold, int numOfParticle, vector<CvMat*>& states, vector<LMDetWind> &detWind, int smoothSize=1, int cannyLow=20, int cannyHigh=40, IplImage* displayImage=NULL);
  void setImage(IplImage* img);
  void buildKdTree(vector<IplImage*>& keyframe, vector<CvMat*>& pose, vector<CvMat*>& keypoint2D, vector<CvMat*>& keypoint3D, vector<CvMat*>& descriptor);  
  IplImage* getImage() { return img_result_; }

protected:
  void findCorrespondenceNN_FLANN(const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs, int numOfKeyframes); // FLANN (approximate nearest neighbor) search
  
  int refineCorrespondenceEpnpRANSAC(
    const vector<int>& ptpairs, 
    vector<CvPoint2D32f>& objOutliers, 
    vector<CvPoint3D32f>& objOutliers3D, 
    vector<CvPoint2D32f>& imgOutliers, 
    vector<CvPoint2D32f>& objInliers, 
    vector<CvPoint3D32f>& objInliers3D, 
    vector<CvPoint2D32f>& imgInliers, 
    CvMat* pmPose
  );
  
  int PF_estimateMultiplePosesEpnp(
    const vector<int>& ptpairs, 
    vector<CvPoint2D32f>& objOutliers, 
    vector<CvPoint3D32f>& objOutliers3D, 
    vector<CvPoint2D32f>& imgOutliers, 
    vector<CvPoint2D32f>& objInliers, 
    vector<CvPoint3D32f>& objInliers3D, 
    vector<CvPoint2D32f>& imgInliers, 
    vector<CvMat*>& states, 
    int numOfParticle
  );

  void DrawDetWind(IplImage *image, int x, int y, int detWindWidth, int detWindHeight, CvScalar scalar, int thickness)
  {
    cvLine(image, cvPoint(x,y), cvPoint(x+detWindWidth,y), scalar, thickness);
    cvLine(image, cvPoint(x+detWindWidth,y), cvPoint(x+detWindWidth, y+detWindHeight), scalar, thickness);
    cvLine(image, cvPoint(x+detWindWidth,y+detWindHeight), cvPoint(x, y+detWindHeight), scalar, thickness);
    cvLine(image, cvPoint(x, y+detWindHeight), cvPoint( x, y), scalar, thickness);
  }

  CvMemStorage* ms_;
  CvSeq *seq_keypoints_;
  CvSeq *seq_descriptors_;

  // only maintain gray images
  IplImage* img_input_;
  IplImage* img_object_;
  IplImage* img_result_;

  CvMat *pose_;
  vector<int> corr_;
  vector<CvPoint2D32f> outliers_img_2d_;
  vector<CvPoint2D32f> outliers_obj_2d_;
  vector<CvPoint3D32f> outliers_obj_3d_;
  vector<CvPoint2D32f> inliers_img_2d_;
  vector<CvPoint2D32f> inliers_obj_2d_;  
  vector<CvPoint3D32f> inliers_obj_3d_;
  CvSURFParams surf_params_;
  CObjectModel* obj_model_;
  CvMat* intrinsic_;
  CvMat* distortion_;
  cv::Mat kfd_;

  // This look up table is used for determining close keyframe
  // row: indices of total keypoints
  // col: indices of the corresponding keyframe
  vector<int> keyframe_lut_;

  vector<CvPoint2D32f> keyframe_keypoints_2d_;
  vector<CvPoint3D32f> keyframe_keypoints_3d_;
  vector<CvPoint2D32f> input_keypoints_2d_;
  int keyframe_idx_;
  vector<IplImage*> keyframe_images_;

  bool verbose_;
  bool display_;
  int draw_type_;

  LFLineFitter lf_;
  LMLineMatcher lm_;
};
