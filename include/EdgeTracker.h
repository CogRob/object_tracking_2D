#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <opencv/cv.h>
#include <TooN/TooN.h>
#include <TooN/SVD.h>       // for SVD
#include <TooN/so3.h>       // for special orthogonal group
#include <TooN/se3.h>       // for special Euclidean group
#include <TooN/wls.h>       // for weighted least square

#include "ObjectModel.h"

using namespace TooN;

class CEdgeTracker
{
public:
  CEdgeTracker(int width, int height, CvMat* intrinsic, int maxd, bool limityrot = false);
  ~CEdgeTracker(void);

  void getEstimatedPoseIRLS(CvMat* pose_cur, CvMat* pose_pre, std::vector<CObjectModel::SamplePoint>& vSamplePt);
  void PF_getJacobianAndError(CvMat* Mprev, std::vector<CObjectModel::SamplePoint>& vSamplePt, CvMat** J, CvMat** e);
  void PF_getError(CvMat* Mprev, std::vector<CObjectModel::SamplePoint>& vSamplePt, CvMat** e);
  Vector<6> calcJacobian(CvPoint3D32f& pts3, CvPoint2D32f& pts2, CvPoint2D32f& ptsnv, double ptsd, const SE3 &E);
  inline CvMat* getPose() { return pose_; }

protected:
  CvMat *pose_;
  int width_;
  int height_;
  CvMat* intrinsic_;
  int maxd_;
  bool limityrot_;
  Matrix<2,2> ja_;
  float alpha_;
};
