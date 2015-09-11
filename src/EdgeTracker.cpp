#include "object_tracking_2D/EdgeTracker.h"
//#include "ObjectModel.h"

CEdgeTracker::CEdgeTracker(int width, int height, CvMat* intrinsic, int maxd, bool limityrot/*=false*/)
  : width_(width)
  , height_(height)
  , pose_(NULL)
  , intrinsic_(NULL)
  , alpha_(64.f)
{
  pose_ = cvCreateMat(4, 4, CV_32F);
  covariance_ = cvCreateMat(6,6,CV_32F);

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

void CEdgeTracker::getEstimatedPoseIRLS(CvMat* pose_cur, CvMat* pose_pre, std::vector<CObjectModel::SamplePoint> &vSamplePt, int vs)
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
        1/(static_cast<double>(alpha_) + abs(vSamplePt[i].dist)/*vSamplePt[i].dist>0 ? vSamplePt[i].dist : -vSamplePt[i].dist)*/)
      );
    }
  }

  wls.compute();

  Vector<6> mu = wls.get_mu();
 /* Matrix<6> inv_cov = wls.get_C_inv();
  cvInvert(covariance_pre,covariance_pre);

  std::cout<<"The covariance from the poses"<<std::endl;
  for(int i=0;i<6;i++)
  {
     for(int j=0;j<6;j++)
     { CV_MAT_ELEM(*covariance_, float, i,j) = CV_MAT_ELEM(*covariance_pre, float, i,j)+inv_cov(i,j);
     }

   }


 cvInvert(covariance_,covariance_);*/
 // std::cout<<std::endl;

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

void CEdgeTracker::getEstimatedPoseIRLS_cov(CvMat* pose_cur, CvMat* pose_pre, std::vector<CObjectModel::SamplePoint> &vSamplePt, int vs,CvMat* covariance_pre)
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
     // std::cout<<calcJacobian(vSamplePt[i].coord3, vSamplePt[i].coord2, vSamplePt[i].nuv, vSamplePt[i].dist, se3_prev)<<"j val"<<1/(static_cast<double>(alpha_) + abs(vSamplePt[i].dist))<<std::endl;
      wls.add_df(
        vSamplePt[i].dist,
        calcJacobian(vSamplePt[i].coord3, vSamplePt[i].coord2, vSamplePt[i].nuv, vSamplePt[i].dist, se3_prev),
        1/(static_cast<double>(alpha_) + abs(vSamplePt[i].dist)/*vSamplePt[i].dist>0 ? vSamplePt[i].dist : -vSamplePt[i].dist)*/)
      );
    }
  }

  wls.compute();

  Vector<6> mu = wls.get_mu();
  Matrix<6> inv_cov = (wls.get_C_inv()) ;

 // std::cout<<"The inv cov "<<inv_cov<<std::endl;

  cvInvert(covariance_pre,covariance_pre);

  std::cout<<"The covariance from the poses"<<std::endl;
  for(int i=0;i<6;i++)
  {
     for(int j=0;j<6;j++)
     { CV_MAT_ELEM(*covariance_, float, i,j) = /*CV_MAT_ELEM(*covariance_pre, float, i,j)+*/inv_cov(i,j);
     }

   }


 cvInvert(covariance_,covariance_);
 // std::cout<<std::endl;

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
  // std::cout<<J[i]<<"THe jacobian"<<temp2[0]<<" "<<ja_[0][0]<<std::endl;
  }

  return J;
}

CvMat* CEdgeTracker::Update( CvMat* J, CvMat* e, int NumOfVisibleSamplePoint, CvMat* covariance_old)
{
  double ct = (double)cvGetTickCount();

  /*if(J == NULL && e == NULL)
  {
    prob_[i] = 0.0f;
    return;
  }*/
 //std::cout<<"in the update step"<<J->rows<<std::endl;

  CvMat* sigma_11 = cvCreateMat(6, 6, CV_32F);
  CvMat* sigma_12 = cvCreateMat(6, J->rows, CV_32F);
  CvMat* sigma_22 = cvCreateMat(J->rows, J->rows, CV_32F);
  CvMat* sigma_22_inv = cvCreateMat(J->rows, J->rows, CV_32F);
  CvMat* J_sigma_11 = cvCreateMat(J->rows, 6, CV_32F);
  CvMat* measure_cov = cvCreateMat(J->rows, J->rows, CV_32F);
  CvMat* Cov = cvCreateMat(6, 6, CV_32F);
  CvMat* sigma_22_inv_e = cvCreateMat(J->rows, 1, CV_32F);
  CvMat* sigma_12_sigma_22_inv = cvCreateMat(6, J->rows, CV_32F);
  CvMat* inc = cvCreateMat(6, 1, CV_32F);
 // std::cout<<"getting covariance"<<std::endl;
 // cvCopy(covariance_old, sigma_11);
  //double val = 1/J->rows;
  cvSetIdentity(sigma_11);
  cvSetIdentity(measure_cov);

  for(int i=0;i<J->rows;i++)
  {    CV_MAT_ELEM(*measure_cov, float, i,i) = (CV_MAT_ELEM(*e, float, i,0))/J->rows;
      if(i<6)
      CV_MAT_ELEM(*sigma_11, float, i,i) = 0.1;

   }



  cvGEMM(sigma_11, J, 1.0, NULL, 0.0, sigma_12, CV_GEMM_B_T); // sigma_12 = sigma_11 * J';
  // get mean_e
  CvScalar mean_e = cvAvg(e);
  //cvSetIdentity(measure_cov);
  std::cout<<"the number of error rows "<<J->rows<<"eror r "<<e->rows<<"cols"<<e->cols<<std::endl;


 // cv::Mat cova = cv::Mat(measure_cov);
 // std::cout<<cova<<std::endl;
  //cvSetIdentity(measure_cov, cvScalar(1));
  cvMatMul(J, sigma_11, J_sigma_11);
  cvGEMM(J_sigma_11, J, 1.0, measure_cov, 1.0, sigma_22, CV_GEMM_B_T); // sigma_22 = J*sigma_11*J' + measure_cov;
  cvInvert(sigma_22, sigma_22_inv);
  cvMatMul(sigma_22_inv, e, sigma_22_inv_e);
  cvMatMul(sigma_12, sigma_22_inv_e, inc);

  cvMatMul(sigma_12, sigma_22_inv, sigma_12_sigma_22_inv);
  cvGEMM(sigma_12_sigma_22_inv, sigma_12, -1.0, sigma_11, 1.0, Cov, CV_GEMM_B_T);
  //cvCopy(&Cov,&covariance_cur);
  //covariance_cur = cv::Mat(Cov);

  std::cout<<"The values for Kalman filtering approach"<<std::endl;
  double covm_data[6][6];
  for(int r=0; r<6; r++)
  {  for(int c=0; c<6; c++)
      { //CV_MAT_ELEM(*covariance_cur, float, r, c) = CV_MAT_ELEM(*Cov, float, r, c);
          covm_data[r][c] = (double)CV_MAT_ELEM(*Cov, float, r, c);
          if(r == c && NumOfVisibleSamplePoint > 0)
             std::cout<<(covm_data[r][c])<<" ";
      }

  }
  std::cout<<std::endl;

  // update on states_pred_
  /*double inc_data[6];
  for(int j=0; j<6; j++)
    inc_data[j] = CV_MAT_ELEM(*inc, float, j, 0);
  Vector<6> inc_vec(inc_data);

  if(limityrot_)
    inc_vec[4] = 0.0; // y-axis angle
  SE3 M_inc = SE3::exp(inc_vec);

  // states_pred_[i] = states_prop_[i] x M_inc x M_rnd
  double covm_data[6][6];
  for(int r=0; r<6; r++)
    for(int c=0; c<6; c++)
      covm_data[r][c] = (double)CV_MAT_ELEM(*Cov, float, r, c);

  Matrix<4> M_prop;
  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      M_prop[r][c] = (double)CV_MAT_ELEM(*states_prop_[i], float, r, c);

  Matrix<4> M_pred = M_prop * M_inc;

  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      CV_MAT_ELEM(*states_pred_[i], float, r, c) = (float)M_pred[r][c];

  // calculate AR velocity
  CvMat* state_inv = cvCreateMat(4, 4, CV_32F);
  cvInvert(states_[i], state_inv, CV_SVD);
  cvMatMul(state_inv, states_pred_[i], ar_vel_[i]);
  cvReleaseMat(&state_inv);*/

  cvReleaseMat(&sigma_11);
  cvReleaseMat(&sigma_12);
  cvReleaseMat(&sigma_22);
  cvReleaseMat(&sigma_22_inv);
  cvReleaseMat(&J_sigma_11);
  cvReleaseMat(&measure_cov);
  //cvReleaseMat(&Cov);
  cvReleaseMat(&sigma_22_inv_e);
  cvReleaseMat(&sigma_12_sigma_22_inv);
  cvReleaseMat(&inc);


  return Cov;
}
