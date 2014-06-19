#include "EdgeTracker.h"
//#include "ObjectModel.h"

CEdgeTracker::CEdgeTracker(int width, int height, CvMat* intrinsic, int maxd, bool limityrot/*=false*/)
  : width_(width)
  , height_(height)
  , pose_(NULL)
  , intrinsic_(NULL)
  , alpha_(64.f)
{
  pose_ = cvCreateMat(4, 4, CV_32F);

  width_ = width;
  height_ = height;

  intrinsic_ = cvCreateMat(3, 3, CV_32F);

  cvCopy(intrinsic, intrinsic_);

  ja_[0][0] = static_cast<double>(CV_MAT_ELEM(*intrinsic_, float, 0, 0)); ja_[0][1] = 0.0;
  ja_[1][1] = static_cast<double>(CV_MAT_ELEM(*intrinsic_, float, 1, 1)); ja_[1][0] = 0.0;
  
  maxd_ = maxd;
  limityrot_ = limityrot;
}

CEdgeTracker::~CEdgeTracker(void)
{
  if(pose_)      cvReleaseMat(&pose_);
  if(intrinsic_) cvReleaseMat(&intrinsic_);
}

void CEdgeTracker::getEstimatedPoseIRLS(CvMat* pose_cur, CvMat* pose_pre, std::vector<CObjectModel::SamplePoint> &vSamplePt)
{
  // use a numerical non-linear optimization (weighted least square) to find pose (P)
  double m_prev[4][4];
  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      m_prev[r][c] = CV_MAT_ELEM(*pose_pre, float, r, c);

  Matrix<4> M(m_prev);
  SE3 se3_prev;
  se3_prev = M;
  
  WLS<6> wls;
  for(int i=0; i<int(vSamplePt.size()); i++)
  {
    if(vSamplePt[i].dist < maxd_)
    {
      // INVERSE 1/(alpha_ + dist)
      wls.add_df(
        vSamplePt[i].dist, 
        calcJacobian(vSamplePt[i].coord3, vSamplePt[i].coord2, vSamplePt[i].nuv, vSamplePt[i].dist, se3_prev), 
        1.0/(static_cast<double>(alpha_) + (vSamplePt[i].dist>0 ? vSamplePt[i].dist : -vSamplePt[i].dist))
      );
    }
  }

  wls.compute();

  Vector<6> mu = wls.get_mu();

  if(limityrot_)
    mu[4] = 0.0;

  SE3 se3_cur;
  se3_cur = se3_prev * SE3::exp(mu);

  Matrix<3> rot = se3_cur.getRot();
  Vector<3> trans = se3_cur.getTrans();

  CV_MAT_ELEM(*pose_cur, float, 0, 0) = static_cast<float>(rot(0,0));
  CV_MAT_ELEM(*pose_cur, float, 0, 1) = static_cast<float>(rot(0,1));
  CV_MAT_ELEM(*pose_cur, float, 0, 2) = static_cast<float>(rot(0,2));
  CV_MAT_ELEM(*pose_cur, float, 0, 3) = static_cast<float>(trans[0]);
  CV_MAT_ELEM(*pose_cur, float, 1, 0) = static_cast<float>(rot(1,0));
  CV_MAT_ELEM(*pose_cur, float, 1, 1) = static_cast<float>(rot(1,1));
  CV_MAT_ELEM(*pose_cur, float, 1, 2) = static_cast<float>(rot(1,2));
  CV_MAT_ELEM(*pose_cur, float, 1, 3) = static_cast<float>(trans[1]);
  CV_MAT_ELEM(*pose_cur, float, 2, 0) = static_cast<float>(rot(2,0));
  CV_MAT_ELEM(*pose_cur, float, 2, 1) = static_cast<float>(rot(2,1));
  CV_MAT_ELEM(*pose_cur, float, 2, 2) = static_cast<float>(rot(2,2));
  CV_MAT_ELEM(*pose_cur, float, 2, 3) = static_cast<float>(trans[2]);
  CV_MAT_ELEM(*pose_cur, float, 3, 0) = 0.0f;
  CV_MAT_ELEM(*pose_cur, float, 3, 1) = 0.0f;
  CV_MAT_ELEM(*pose_cur, float, 3, 2) = 0.0f;
  CV_MAT_ELEM(*pose_cur, float, 3, 3) = 1.0f;
}

void CEdgeTracker::PF_getJacobianAndError(CvMat* Mprev, std::vector<CObjectModel::SamplePoint>& vSamplePt, CvMat** J, CvMat** e)
{
  double ct = (double)cvGetTickCount();
  // calculate Jacobian matrix 'J' and error vector 'e'

  double dist_max = maxd_;
  // first count the number of valid sample points
  int nov = 0;
  for(int i=0; i<int(vSamplePt.size()); i++)
    if(vSamplePt[i].dist < dist_max) // only valid points
      nov++;

  if(nov < 6)
    return;

  // then allocate matrices
  *J = cvCreateMat(nov, 6, CV_32F);
  *e = cvCreateMat(nov, 1, CV_32F);
  // stack all valid Jocabian and error
  nov = 0;
  Matrix<4> M_prev;
  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      M_prev[r][c] = CV_MAT_ELEM(*Mprev, float, r, c);

  SE3 SE3_prev;
  SE3_prev = M_prev;

  for(int i=0; i<int(vSamplePt.size()); i++)
  {
    if(vSamplePt[i].dist < dist_max) // only valid points
    {
      CV_MAT_ELEM(**e, float, nov, 0) = (float)vSamplePt[i].dist;
      Vector<6> jacob = calcJacobian(vSamplePt[i].coord3, vSamplePt[i].coord2, vSamplePt[i].nuv, vSamplePt[i].dist, SE3_prev);
      for(int c=0; c<6; c++)
        CV_MAT_ELEM(**J, float, nov, c) = jacob[c];
      nov++;
    }
  }
}

void CEdgeTracker::PF_getError(CvMat* Mprev, std::vector<CObjectModel::SamplePoint>& vSamplePt, CvMat** e)
{
  double ct = (double)cvGetTickCount();
  // calculate error vector 'e'

  double dist_max = maxd_;
  // first count the number of valid sample points
  int nov = 0;
  for(int i=0; i<int(vSamplePt.size()); i++)
  {
    if(vSamplePt[i].dist < dist_max) // only valid points
    {
      nov++;
    }
  }
  if(nov == 0)
    return;
  // then allocate matrices
  *e = cvCreateMat(nov, 1, CV_32F);
  // stack all valid Jocabian and error
  nov = 0;
  Matrix<4> M_prev;
  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      M_prev[r][c] = CV_MAT_ELEM(*Mprev, float, r, c);
  SE3 SE3_prev;
  SE3_prev = M_prev;

  for(int i=0; i<int(vSamplePt.size()); i++)
  {
    if(vSamplePt[i].dist < dist_max) // only valid points
    {
      CV_MAT_ELEM(**e, float, nov, 0) = (float)vSamplePt[i].dist;
      nov++;
    }
  }
}

Vector<6> CEdgeTracker::calcJacobian(CvPoint3D32f& pts3, CvPoint2D32f& pts2, CvPoint2D32f& ptsnv, double ptsd, const SE3& E)
{
  Vector<4> vpts3; // 3D points
  Vector<3> vpts2; // 2D points
  Vector<2> vptsn; // normal
  Vector<6> J;

  // Initialize values
  vpts3 = pts3.x, pts3.y, pts3.z, 1.0;
  vpts2 = pts2.x, pts2.y, 1.0;
  vptsn = ptsnv.x, ptsnv.y;

  for(int i = 0; i < 6; i++)
  {
    Vector<4> cam_coord = E*vpts3;
    Vector<4> temp = E*SE3::generator_field(i, vpts3);
    Vector<2> temp2 = temp.slice<0,2>()/cam_coord[2] - cam_coord.slice<0,2>() * (temp[2]/cam_coord[2]/cam_coord[2]);
    J[i] = vptsn*ja_*temp2; // Jc is not required in here.
  }

  return J;
}
