#include "object_tracking_2D/HomoTransform.h"
#include <math.h>

#define eps 1e-13

CHomoTransform::CHomoTransform(void)
{
}

CHomoTransform::~CHomoTransform(void)
{
}

void CHomoTransform::tr2rpy(CvMat *M, float r, float p, float y)
{
  r = atan2(-CV_MAT_ELEM(*M, float, 1, 2), CV_MAT_ELEM(*M, float, 2, 2)); // roll
  float sr = sin(r);
  float cr = cos(r);
  p = atan2(CV_MAT_ELEM(*M, float, 0, 2), cr * CV_MAT_ELEM(*M, float, 2, 2) - sr * CV_MAT_ELEM(*M, float, 1, 2)); // pitch
  y = atan2(-CV_MAT_ELEM(*M, float, 0, 1), CV_MAT_ELEM(*M, float, 0, 0));
}

void CHomoTransform::rpy2tr(float r, float p, float y, CvMat *M)
{
  CvMat *Mr = cvCreateMat(3, 3, CV_32F);
  CvMat *Mp = cvCreateMat(3, 3, CV_32F);
  CvMat *My = cvCreateMat(3, 3, CV_32F);

  cvSetIdentity(M); // initialize
  cvSetIdentity(Mr);
  cvSetIdentity(Mp);
  cvSetIdentity(My);

  // rotx(roll)
  float sr = sin(r);
  float cr = cos(r);
  CV_MAT_ELEM(*Mr, float, 1, 1) = cr;
  CV_MAT_ELEM(*Mr, float, 1, 2) = -sr;
  CV_MAT_ELEM(*Mr, float, 2, 1) = sr;
  CV_MAT_ELEM(*Mr, float, 2, 2) = cr;

  // roty(pitch)
  float sp = sin(p);
  float cp = cos(p);
  CV_MAT_ELEM(*Mp, float, 0, 0) = cp;
  CV_MAT_ELEM(*Mp, float, 0, 2) = sp;
  CV_MAT_ELEM(*Mp, float, 2, 0) = -sp;
  CV_MAT_ELEM(*Mp, float, 2, 2) = cp;

  // rotz(yaw)
  float sy = sin(y);
  float cy = cos(y);
  CV_MAT_ELEM(*My, float, 0, 0) = cy;
  CV_MAT_ELEM(*My, float, 0, 1) = -sy;
  CV_MAT_ELEM(*My, float, 1, 0) = sy;
  CV_MAT_ELEM(*My, float, 1, 1) = cy;

  cvMatMul(Mr, Mp, Mp);
  cvMatMul(Mp, My, My);

  for(int j=0; j<3; j++)
  {
    for(int i=0; i<3; i++)
    {
      CV_MAT_ELEM(*M, float, i, j) = CV_MAT_ELEM(*My, float, i, j);
    }
  }

  cvReleaseMat(&Mr);
  cvReleaseMat(&Mp);
  cvReleaseMat(&My);
}

void CHomoTransform::tr2eul(CvMat *M, float phi, float theta, float psi)
{
  float sp, cp;
  if(abs(CV_MAT_ELEM(*M, float, 0, 2)) < eps && abs(CV_MAT_ELEM(*M, float, 1, 2)) < eps)
  {
    // singularity
    phi = 0;
    sp = 0;
    cp = 1;
    theta = atan2(cp*CV_MAT_ELEM(*M, float, 0, 2) + sp*CV_MAT_ELEM(*M, float, 1, 2), CV_MAT_ELEM(*M, float, 2, 2));
    psi = atan2(-sp*CV_MAT_ELEM(*M, float, 0, 0) + cp*CV_MAT_ELEM(*M, float, 1, 0), -sp*CV_MAT_ELEM(*M, float, 0, 1) + cp*CV_MAT_ELEM(*M, float, 1, 1));
  }
  else
  {
    phi = atan2(CV_MAT_ELEM(*M, float, 1, 2), CV_MAT_ELEM(*M, float, 0, 2));
    sp = sin(phi);
    cp = cos(phi);
    theta = atan2(cp*CV_MAT_ELEM(*M, float, 0, 2) + sp*CV_MAT_ELEM(*M, float, 1, 2), CV_MAT_ELEM(*M, float, 2, 2));
    psi = atan2(-sp*CV_MAT_ELEM(*M, float, 0, 0) + cp * CV_MAT_ELEM(*M, float, 1, 0), -sp*CV_MAT_ELEM(*M, float, 0, 1) + cp*CV_MAT_ELEM(*M, float, 1, 1));
  }
}

void CHomoTransform::eul2tr(float phi, float theta, float psi, CvMat *M)
{
  CvMat *Mph = cvCreateMat(3, 3, CV_32F);
  CvMat *Mth = cvCreateMat(3, 3, CV_32F);
  CvMat *Mps = cvCreateMat(3, 3, CV_32F);

  // initialize to identity matrices
  cvSetIdentity(M);
  cvSetIdentity(Mph);
  cvSetIdentity(Mth);
  cvSetIdentity(Mps);

  // rotz(phi)
  float sph = sin(phi);
  float cph = cos(phi);
  CV_MAT_ELEM(*Mph, float, 0, 0) =  cph;
  CV_MAT_ELEM(*Mph, float, 0, 1) = -sph;
  CV_MAT_ELEM(*Mph, float, 1, 0) =  sph;
  CV_MAT_ELEM(*Mph, float, 1, 1) =  cph;

  // roty(theta)
  float sth = sin(theta);
  float cth = cos(theta);
  CV_MAT_ELEM(*Mth, float, 0, 0) =  cth;
  CV_MAT_ELEM(*Mth, float, 0, 2) =  sth;
  CV_MAT_ELEM(*Mth, float, 2, 0) = -sth;
  CV_MAT_ELEM(*Mth, float, 2, 2) =  cth;

  // rotz(psi)
  float sps = sin(psi);
  float cps = cos(psi);
  CV_MAT_ELEM(*Mps, float, 0, 0) =  cps;
  CV_MAT_ELEM(*Mps, float, 0, 1) = -sps;
  CV_MAT_ELEM(*Mps, float, 1, 0) =  sps;
  CV_MAT_ELEM(*Mps, float, 1, 1) =  cps;

  cvMatMul(Mph, Mth, Mth);
  cvMatMul(Mth, Mps, Mps);

  for(int j=0; j<3; j++)
  {
    for(int i=0; i<3; i++)
    {
      CV_MAT_ELEM(*M, float, i, j) = CV_MAT_ELEM(*Mps, float, i, j);
    }
  }

  cvReleaseMat(&Mph);
  cvReleaseMat(&Mth);
  cvReleaseMat(&Mps);
}

void CHomoTransform::tr2rpy_kuka(CvMat *M, float r, float p, float y)
{
  p = atan2(-CV_MAT_ELEM(*M, float, 2, 0), sqrt(CV_MAT_ELEM(*M, float, 0, 0)*CV_MAT_ELEM(*M, float, 0, 0)+CV_MAT_ELEM(*M, float, 1, 0)*CV_MAT_ELEM(*M, float, 1, 0)));

  float sp = sin(p);
  float cp = cos(p);

  if(3.14159/2.-eps < abs(p) && abs(p) < 3.14159/2.+eps)
  {
    y = 0.;
  }
  else
  {
    y = atan2(CV_MAT_ELEM(*M, float, 1, 0)/cp, CV_MAT_ELEM(*M, float, 0, 0)/cp);
  }

  if(3.14159/2.-eps < abs(p) && abs(p) < 3.14159/2.+eps)
  {
    r = p/abs(p)*atan2(CV_MAT_ELEM(*M, float, 0, 1), CV_MAT_ELEM(*M, float, 1, 1));
  }
  else
  {
    r = atan2(CV_MAT_ELEM(*M, float, 2, 1)/cp, CV_MAT_ELEM(*M, float, 2, 2)/cp);
  }
}

void CHomoTransform::rpy2tr_kuka(float r, float p, float y, CvMat *M)
{
  CvMat *Mr = cvCreateMat(3, 3, CV_32F);
  CvMat *Mp = cvCreateMat(3, 3, CV_32F);
  CvMat *My = cvCreateMat(3, 3, CV_32F);

  cvSetIdentity(M); // initialize
  cvSetIdentity(Mr);
  cvSetIdentity(Mp);
  cvSetIdentity(My);

  // rotx(roll)
  float sr = sin(r);
  float cr = cos(r);
  CV_MAT_ELEM(*Mr, float, 1, 1) = cr;
  CV_MAT_ELEM(*Mr, float, 1, 2) = -sr;
  CV_MAT_ELEM(*Mr, float, 2, 1) = sr;
  CV_MAT_ELEM(*Mr, float, 2, 2) = cr;

  // roty(pitch)
  float sp = sin(p);
  float cp = cos(p);
  CV_MAT_ELEM(*Mp, float, 0, 0) = cp;
  CV_MAT_ELEM(*Mp, float, 0, 2) = sp;
  CV_MAT_ELEM(*Mp, float, 2, 0) = -sp;
  CV_MAT_ELEM(*Mp, float, 2, 2) = cp;

  // rotz(yaw)
  float sy = sin(y);
  float cy = cos(y);
  CV_MAT_ELEM(*My, float, 0, 0) = cy;
  CV_MAT_ELEM(*My, float, 0, 1) = -sy;
  CV_MAT_ELEM(*My, float, 1, 0) = sy;
  CV_MAT_ELEM(*My, float, 1, 1) = cy;

  //cvMatMul(Mr, Mp, Mp);
  //cvMatMul(Mp, My, My);
  cvMatMul(My, Mp, Mp);
  cvMatMul(Mp, Mr, Mr);

  for(int j=0; j<3; j++)
  {
    for(int i=0; i<3; i++)
    {
      CV_MAT_ELEM(*M, float, i, j) = CV_MAT_ELEM(*Mr, float, i, j);
    }
  }

  cvReleaseMat(&Mr);
  cvReleaseMat(&Mp);
  cvReleaseMat(&My);
}

float CHomoTransform::tr3d(CvMat *X, CvMat *Y, CvMat *H)
{
  // X: N x 3
  // Y: N x 3

  // mean correct
  CvPoint3D32f Xm, Ym;
  Xm.x = Xm.y = Xm.z = Ym.x = Ym.y = Ym.z = 0.0f;
  for(int i=0; i<X->rows; i++)
  {
    Xm.x += CV_MAT_ELEM(*X, float, i, 0);
    Xm.y += CV_MAT_ELEM(*X, float, i, 1);
    Xm.z += CV_MAT_ELEM(*X, float, i, 2);

    Ym.x += CV_MAT_ELEM(*Y, float, i, 0);
    Ym.y += CV_MAT_ELEM(*Y, float, i, 1);
    Ym.z += CV_MAT_ELEM(*Y, float, i, 2);
  }
  Xm.x /= X->rows;
  Xm.y /= X->rows;
  Xm.z /= X->rows;

  Ym.x /= Y->rows;
  Ym.y /= Y->rows;
  Ym.z /= Y->rows;

  CvMat *X_ = cvCloneMat(X);
  CvMat *Y_ = cvCloneMat(Y);

  for(int i=0; i<X->rows; i++)
  {
    CV_MAT_ELEM(*X_, float, i, 0) -= Xm.x;
    CV_MAT_ELEM(*X_, float, i, 1) -= Xm.y;
    CV_MAT_ELEM(*X_, float, i, 2) -= Xm.z;

    CV_MAT_ELEM(*Y_, float, i, 0) -= Ym.x;
    CV_MAT_ELEM(*Y_, float, i, 1) -= Ym.y;
    CV_MAT_ELEM(*Y_, float, i, 2) -= Ym.z;
  }

  // calculate best rotation using algorithm 12.4.1 from
  // G. H. Golub and C. F. van Loan, "Matrix Computations"
  // 2nd Edition, Baltimore: Johns Hopkins, 1989, p. 582.
  //CvMat *Xt = cvCloneMat(X_);
  CvMat *Xt = cvCreateMat(X_->cols, X_->rows, CV_32F);
  cvTranspose(X_, Xt);
  CvMat *XtY = cvCreateMat(3, 3, CV_32F);
  cvMatMul(Xt, Y_, XtY);

  CvMat *U = cvCreateMat(3, 3, CV_32F);
  CvMat *W = cvCreateMat(3, 3, CV_32F);
  CvMat *V = cvCreateMat(3, 3, CV_32F);
  CvMat *Vt = cvCreateMat(3, 3, CV_32F);

  cvSVD(XtY, W, U, V);

  CvMat *R = cvCreateMat(3, 3, CV_32F);
  cvTranspose(V, Vt);
  cvMatMul(U, Vt, R);

  // determinant check
  if(cvDet(R) < 0) // det(R) == -1
  {
    //cout << "det(R) is -1" << endl;
    CV_MAT_ELEM(*V, float, 0, 2) = -1*CV_MAT_ELEM(*V, float, 0, 2);
    CV_MAT_ELEM(*V, float, 1, 2) = -1*CV_MAT_ELEM(*V, float, 1, 2);
    CV_MAT_ELEM(*V, float, 2, 2) = -1*CV_MAT_ELEM(*V, float, 2, 2);
    cvTranspose(V, Vt);
    cvMatMul(U, Vt, R);
  }

  // solve for the translation vector
  CvMat *Mx = cvCreateMat(1, 3, CV_32F);
  CvMat *Mt = cvCreateMat(1, 3, CV_32F);
  CV_MAT_ELEM(*Mx, float, 0, 0) = Xm.x;
  CV_MAT_ELEM(*Mx, float, 0, 1) = Xm.y;
  CV_MAT_ELEM(*Mx, float, 0, 2) = Xm.z;

  cvMatMul(Mx, R, Mx);
  CV_MAT_ELEM(*Mt, float, 0, 0) = Ym.x - CV_MAT_ELEM(*Mx, float, 0, 0);
  CV_MAT_ELEM(*Mt, float, 0, 1) = Ym.y - CV_MAT_ELEM(*Mx, float, 0, 1);
  CV_MAT_ELEM(*Mt, float, 0, 2) = Ym.z - CV_MAT_ELEM(*Mx, float, 0, 2);

  // update to 'H'
  cvSetIdentity(H);
  cvTranspose(R, R); // shoud be transposed
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
    {
      CV_MAT_ELEM(*H, float, i, j) = CV_MAT_ELEM(*R, float, i, j);
    }
  }
  CV_MAT_ELEM(*H, float, 0, 3) = CV_MAT_ELEM(*Mt, float, 0, 0);
  CV_MAT_ELEM(*H, float, 1, 3) = CV_MAT_ELEM(*Mt, float, 0, 1);
  CV_MAT_ELEM(*H, float, 2, 3) = CV_MAT_ELEM(*Mt, float, 0, 2);

  cvReleaseMat(&X_);
  cvReleaseMat(&Y_);
  cvReleaseMat(&XtY);
  cvReleaseMat(&U);
  cvReleaseMat(&W);
  cvReleaseMat(&V);
  cvReleaseMat(&Vt);
  cvReleaseMat(&R);
  cvReleaseMat(&Mx);
  cvReleaseMat(&Mt);

  return 0.0;
}

