#pragma once

#include <opencv/cv.h>

class CHomoTransform
{
public:
  CHomoTransform(void);
  ~CHomoTransform(void);

  void tr2rpy(CvMat *M, float r, float p, float y);
  void rpy2tr(float r, float p, float y, CvMat *M);

  void tr2eul(CvMat *M, float phi, float theta, float psi);
  void eul2tr(float phi, float theta, float psi, CvMat *M);

  void tr2rpy_kuka(CvMat *M, float r, float p, float y);
  void rpy2tr_kuka(float r, float p, float y, CvMat *M);

  float tr3d(CvMat *X, CvMat *Y, CvMat *H);
};
