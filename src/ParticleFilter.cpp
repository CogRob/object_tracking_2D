#include "object_tracking_2D/ParticleFilter.h"
#include <assert.h>
#include "object_tracking_2D/randn.h"
#include <time.h>

CParticleFilter::CParticleFilter(int NumOfParticle/*=1*/, float AR_param/*=0.0*/, bool limitYRotation/*=false*/)
  : verbose_(false)
{
  num_particle_ = NumOfParticle;

  mean_state_ = cvCreateMat(4, 4, CV_32F);

  states_.resize(num_particle_);
  states_prev_.resize(num_particle_);
  states_pred_.resize(num_particle_);
  states_prop_.resize(num_particle_);
  states_opt_.resize(num_particle_);
  ar_vel_.resize(num_particle_);
  ar_vel_opt_.resize(num_particle_);
  weight_.resize(num_particle_);
  weight_opt_.resize(num_particle_);
  prob_.resize(num_particle_);
  prob_opt_.resize(num_particle_);
  eta_.resize(num_particle_);
  state_cov_.resize(num_particle_);

  for(int i=0; i<num_particle_; i++)
  {
    states_[i] = cvCreateMat(4, 4, CV_32F);
    states_prev_[i] = cvCreateMat(4, 4, CV_32F);
    states_pred_[i] = cvCreateMat(4, 4, CV_32F);
    states_prop_[i] = cvCreateMat(4, 4, CV_32F);
    states_opt_[i] = cvCreateMat(4, 4, CV_32F);
    ar_vel_[i] = cvCreateMat(4, 4, CV_32F);
    ar_vel_opt_[i] = cvCreateMat(4, 4, CV_32F);
    state_cov_[i] = cvCreateMat(6, 6, CV_32F);
  }
  ar_param_ = AR_param;
  //std::cout<<"AR param"<<ar_param_<<std::endl;
  ar_param_=0.0;
  dt_ = (float)(1./30.);

  neff_ = 0.0f;
  limityrot_ = limitYRotation;

  srand((unsigned int)time(NULL)); // for random noise
}

CParticleFilter::~CParticleFilter(void)
{
  cvReleaseMat(&mean_state_);
  //cvReleaseMat(&state_cov_);
  assert(num_particle_ == states_.size());
  assert(num_particle_ == states_prev_.size());
  assert(num_particle_ == states_pred_.size());
  assert(num_particle_ == states_prop_.size());
  assert(num_particle_ == states_opt_.size());
  assert(num_particle_ == ar_vel_.size());
  assert(num_particle_ == ar_vel_opt_.size());
  assert(num_particle_ == weight_.size());
  assert(num_particle_ == weight_opt_.size());
  assert(num_particle_ == prob_.size());
  assert(num_particle_ == prob_opt_.size());
  assert(num_particle_ == eta_.size());
  if(num_particle_)
  {
    for(int i=0; i<(int)states_.size(); i++)
    {
      cvReleaseMat(&states_[i]);
      cvReleaseMat(&states_prev_[i]);
      cvReleaseMat(&states_pred_[i]);
      cvReleaseMat(&states_prop_[i]);
      cvReleaseMat(&states_opt_[i]);
      cvReleaseMat(&ar_vel_[i]);
      cvReleaseMat(&ar_vel_opt_[i]);
      cvReleaseMat(&state_cov_[i]);
    }
  }
  states_.clear();
  states_prev_.clear();
  states_pred_.clear();
  states_prop_.clear();
  states_opt_.clear();
  ar_vel_.clear();
  ar_vel_opt_.clear();
  weight_.clear();
  weight_opt_.clear();
  prob_.clear();
  prob_opt_.clear();
  eta_.clear();
  state_cov_.clear();

}

void CParticleFilter::Init(CvMat* X/*=NULL*/)
{
  if(X)
    for(int i=0; i<num_particle_; i++)
    {
      cvCopy(X, states_[i]);
      cvCopy(X, states_prev_[i]);
    }
  else
    for(int i=0; i<num_particle_; i++)
    {
      cvSetIdentity(states_[i]);
      cvSetIdentity(states_prev_[i]);
    }

    for(int i=0; i<num_particle_; i++)
    {
      cvSetIdentity(ar_vel_[i]);
      cvSetIdentity(ar_vel_opt_[i]);
      weight_[i] = 0.01f ;//(float)1/num_particle_;
      weight_opt_[i] = 0.01f; //1/num_particle_;
      prob_[i] = 0.0f;
      prob_opt_[i] = 0.0f;
      eta_[i] = 0.0f;
      cvSetIdentity(state_cov_[i], cvRealScalar(dt_));
    }

    cvSetIdentity(mean_state_);
    //cvSetIdentity(state_cov_, cvRealScalar(dt_));
}

void CParticleFilter::Init(int i, CvMat* X/*=NULL*/)
{
  if(X)
  {
    cvCopy(X, states_[i]);
    cvCopy(X, states_prev_[i]);
  }
  else
  {
    cvSetIdentity(states_[i]);
    cvSetIdentity(states_prev_[i]);
  }

  cvSetIdentity(ar_vel_[i]);
  cvSetIdentity(ar_vel_opt_[i]);
  weight_[i] = (float)1/num_particle_;
  weight_opt_[i] = (float)1/num_particle_;
  prob_[i] = 0.0f;
  prob_opt_[i] = 0.0f;
  eta_[i] = 0.0f;

  cvSetIdentity(mean_state_);
  cvSetIdentity(state_cov_[i], cvRealScalar(dt_));
}

void CParticleFilter::Propagate(float noiseRateLow, float noiseRateHigh, bool bCompAR/*=false*/)
{
  double m[4][4];

  SE3 M_SE3;
  Vector<6> M_se3;
  CvMat* M_ar = cvCreateMat(4, 4, CV_32F);
  CvMat* M_r = cvCreateMat(4, 4, CV_32F);
  cvSetIdentity(M_ar);
  cvSetIdentity(M_r);
  //std::cout<<num_particle_<<"ar particles";
  for(int i=0; i<num_particle_; i++)
  {
    for(int r=0; r<4; r++)
     { for(int c=0; c<4; c++)
       { m[r][c] = CV_MAT_ELEM(*ar_vel_[i], float, r, c);
        //if (r ==c) std::cout<<m[r][c]<<" ";
        }
     }
    //std::cout<<endl;*/

    Matrix<4> M(m);
    M_SE3 = M;
    M_se3 = SE3::ln(M_SE3)*ar_param_;
    M_SE3 = SE3::exp(M_se3);

    double rot[3][3];
    Matrix<3> Rot(rot);
    Rot = M_SE3.getRot();
    Vector<3> Trans = M_SE3.getTrans();

    for(int r=0; r<3; r++)
      for(int c=0; c<3; c++)
        CV_MAT_ELEM(*M_ar, float, r, c) = float(Rot(r,c));
    for(int r=0; r<3; r++)
      CV_MAT_ELEM(*M_ar, float, r, 3) = float(Trans[r]);

    //std::cout<<"AR"<<cv::Mat(M_ar)<<std::endl;


    if(!bCompAR)
      cvSetIdentity(M_ar);

    cvMatMul(states_[i], M_ar, states_prop_[i]);

    // add Gaussian noise
    Vector<6> randn_vec;
    for(int r=0; r<6; r++)
      randn_vec[r] = randn_notrig(0.0,0.01,i); // generate normal random number without using tirangular fucntions

    /*for(int r=0; r<6; r++)
    {    for(int c=0; c<6; c++)
        {    if(r == c)
             { // std:://cout<<temp_covar.at<float>(r,c)<<" ";
                randn_vec[r] = randn_notrig(0.0,  CV_MAT_ELEM(*state_cov_[i], float,r,c),i); // generate normal random number without using tirangular fucntions
             }
        }
     }//*/



    if(limityrot_)
      randn_vec[4] = 0.0; // y-axis angle

    // Two Gaussian Mode
    SE3 M_rnd;
    if(num_particle_ > 1 && i < num_particle_/2)
      M_rnd = SE3::exp(randn_vec*noiseRateLow);
    else if(num_particle_ > 1 && i >= num_particle_/2)
      M_rnd = SE3::exp(randn_vec*noiseRateHigh );
    else
      M_rnd = SE3::exp(randn_vec*0.0);

    Rot = M_rnd.getRot();
    Trans = M_rnd.getTrans();
  // std::cout<<"Rand movement old"<<Rot<<Trans<<std::endl;

    for(int r=0; r<3; r++)
      for(int c=0; c<3; c++)
        CV_MAT_ELEM(*M_r, float, r, c) = float(Rot(r,c));
    for(int r=0; r<3; r++)
      CV_MAT_ELEM(*M_r, float, r, 3) = float(Trans[r]);

    cvMatMul(states_prop_[i], M_r, states_prop_[i]);
    //cv::Mat temp = cv::Mat(M_ar);
   // std::cout<<cv::Mat(M_ar)<<std::endl;
   // std::cout<<cv::Mat(M_r)<<std::endl;
   // std::cout<<cv::Mat(states_prop_[i])<<std::endl;

  }
  cvReleaseMat(&M_ar);
  cvReleaseMat(&M_r);
}

void CParticleFilter::Update(int i, CvMat* J, CvMat* e, int NumOfVisibleSamplePoint, float lamda/*=1.0f*/)
{
  double ct = (double)cvGetTickCount();

  if(J == NULL && e == NULL)
  {
    prob_[i] = 0.0f;
    return;
  }

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

  //cvCopy(state_cov_, sigma_11);
  cvSetIdentity(sigma_11, cvScalar(1));
  cvGEMM(sigma_11, J, 1.0, NULL, 0.0, sigma_12, CV_GEMM_B_T); // sigma_12 = sigma_11 * J';
  // get mean_e
  CvScalar mean_e = cvAvg(e);
  cvSetIdentity(measure_cov, cvRealScalar(mean_e.val[0]*mean_e.val[0]));
  cvMatMul(J, sigma_11, J_sigma_11);
  cvGEMM(J_sigma_11, J, 1.0, measure_cov, 1.0, sigma_22, CV_GEMM_B_T); // sigma_22 = J*sigma_11*J' + measure_cov;
  cvInvert(sigma_22, sigma_22_inv);
  cvMatMul(sigma_22_inv, e, sigma_22_inv_e);
  cvMatMul(sigma_12, sigma_22_inv_e, inc);

  cvMatMul(sigma_12, sigma_22_inv, sigma_12_sigma_22_inv);
  cvGEMM(sigma_12_sigma_22_inv, sigma_12, -1.0, sigma_11, 1.0, Cov, CV_GEMM_B_T);

  cvCopy(Cov,state_cov_[i]);
  double covm_data[6][6];
  for(int r=0; r<6; r++)
  {  for(int c=0; c<6; c++)
      {covm_data[r][c] = (double)CV_MAT_ELEM(*Cov, float, r, c);
    //  if(r == c) //std::cout<<covm_data[r][c];
      }
   }
 // std::cout<<std::endl;

     //   }
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
  cvReleaseMat(&Cov);
  cvReleaseMat(&sigma_22_inv_e);
  cvReleaseMat(&sigma_12_sigma_22_inv);
  cvReleaseMat(&inc);
}

void CParticleFilter::Update_IRLS(int i, CvMat* J, CvMat* e, int NumOfVisibleSamplePoint)
{
    double ct = (double)cvGetTickCount();

    if(J == NULL && e == NULL)
    {
      prob_[i] = 0.0f;
      return;
    }

    // J: N x 6 matrix
    // e: N x 1 matrix (vector)

 //std::cout<<cv::Mat(e).t()<<std::endl;
    assert(J->cols == 6);
    assert(J->rows == e->rows);

    int N = J->rows;

    float c = 64.f;

    CvMat* W = cvCreateMat(N, N, CV_32F);
    CvMat* Jt_W = cvCreateMat(6, N, CV_32F);
    CvMat* Jt_W_J = cvCreateMat(6, 6, CV_32F);
    CvMat* Jt_W_J_inv = cvCreateMat(6, 6, CV_32F);
    CvMat* Jt_W_e = cvCreateMat(6, 1, CV_32F);
    CvMat* inc = cvCreateMat(6, 1, CV_32F);

    cvSetIdentity(W);
    for(int j=0; j<N; j++)
    {
      CV_MAT_ELEM(*W, float, j, j) = 1.0f/(c+CV_MAT_ELEM(*e, float, j, 0));

    }

    cvGEMM(J, W, 1.0, NULL, 0.0, Jt_W, CV_GEMM_A_T); // J'*W
    cvGEMM(Jt_W, J, 1.0, NULL, 0.0, Jt_W_J); // J'*W*J
    cvInvert(Jt_W_J, Jt_W_J_inv); // inv(J'*W*J)
    cvGEMM(Jt_W, e, 1.0, NULL, 0.0, Jt_W_e); // J'*W*e
    cvMatMul(Jt_W_J_inv, Jt_W_e, inc);

    cvCopy(Jt_W_J_inv,state_cov_[i]);
    //std:://cout<<cv::Mat(Jt_W_J_inv)<<std::endl;

    CvScalar mean_e = cvAvg(e);
    // Update on states_pred_
    double inc_data[6];
    for(int j=0; j<6; j++)
      inc_data[j] = 2.0*CV_MAT_ELEM(*inc, float, j, 0);

    if(limityrot_)
      inc_data[4] = 0.0; // y-axis angle

    Vector<6> inc_vec(inc_data);
    SE3 M_inc = SE3::exp(inc_vec);

    Matrix<4> M_prop;
    for(int r=0; r<4; r++)
      for(int c=0; c<4; c++)
        M_prop[r][c] = (double)CV_MAT_ELEM(*states_prop_[i], float, r, c);

    // No IRLS
    Matrix<4> M_pred = M_prop;
    // Optimized with IRLS
    Matrix<4> M_opt = M_prop * M_inc;

    for(int r=0; r<4; r++)
    {
      for(int c=0; c<4; c++)
      {
        CV_MAT_ELEM(*states_pred_[i], float, r, c) = (float)M_pred[r][c];
        CV_MAT_ELEM(*states_opt_[i], float, r, c) = (float)M_opt[r][c];
  //       std::cout<<M_opt[r][c]<<" ";
      }
    }


 //   std::cout<<std::endl;
    // Calculate AR velocity
    CvMat* state_inv = cvCreateMat(4, 4, CV_32F);
    cvInvert(states_[i], state_inv, CV_SVD);
    cvMatMul(state_inv, states_pred_[i], ar_vel_[i]);
    cvMatMul(state_inv, states_opt_[i], ar_vel_opt_[i]);
    /*for(int cur_row = 0; cur_row<4; cur_row++)
              {
                 printf("[%1.3f %1.3f %1.3f %1.3f]\n", CV_MAT_ELEM(*states_opt_[i], float, cur_row, 0), CV_MAT_ELEM(*states_opt_[i], float, cur_row, 1) ,CV_MAT_ELEM(*states_opt_[i], float, cur_row, 2), CV_MAT_ELEM(*states_opt_[i], float, cur_row, 3));
          }*/
    cvReleaseMat(&state_inv);

    cvReleaseMat(&W);
    cvReleaseMat(&Jt_W);
    cvReleaseMat(&Jt_W_J);
    cvReleaseMat(&Jt_W_J_inv);
    cvReleaseMat(&Jt_W_e);
    cvReleaseMat(&inc);



 /* double ct = (double)cvGetTickCount();

  if(J == NULL && e == NULL)
  {
    prob_[i] = 0.0f;
    return;
  }

  // J: N x 6 matrix
  // e: N x 1 matrix (vector)
  assert(J->cols == 6);
  assert(J->rows == e->rows);


  int N = J->rows;
 // std::cout<<"in the uodate IROLS"<< N<<std::endl;


  float c = 64.f;

  CvMat* W = cvCreateMat(N, N, CV_32F);
  CvMat* Jt_W = cvCreateMat(6, N, CV_32F);
  CvMat* Jt_W_J = cvCreateMat(6, 6, CV_32F);
  CvMat* Jt_W_J_inv = cvCreateMat(6, 6, CV_32F);
  CvMat* Jt_W_e = cvCreateMat(6, 1, CV_32F);
  CvMat* inc = cvCreateMat(6, 1, CV_32F);

  cvSetIdentity(W);
  for(int j=0; j<N; j++)
  {
   // CV_MAT_ELEM(*W, float, j, j) = (96.0f-(0.5*CV_MAT_ELEM(*e, float, j, 0)))/((c+CV_MAT_ELEM(*e, float, j, 0))*(c-CV_MAT_ELEM(*e, float, j, 0)));
    CV_MAT_ELEM(*W, float, j, j) = 1.0f/((c+CV_MAT_ELEM(*e, float, j, 0)));
  }

  //cvCopy(state_cov_,W);
  cvGEMM(J, W, 1.0, NULL, 0.0, Jt_W, CV_GEMM_A_T); // J'*W
  cvGEMM(Jt_W, J, 1.0, NULL, 0.0, Jt_W_J); // J'*W*J
  cvInvert(Jt_W_J, Jt_W_J_inv); // inv(J'*W*J)
  cvGEMM(Jt_W, e, 1.0, NULL, 0.0, Jt_W_e); // J'*W*e
  cvMatMul(Jt_W_J_inv, Jt_W_e, inc);

  cvCopy(Jt_W_J_inv,state_cov_[i]);

 // std::cout<<"Cov"<<cv::Mat(Jt_W_J_inv).diag()<<std::endl;

  CvScalar mean_e = cvAvg(e);
  // Update on states_pred_
  double inc_data[6];
  for(int j=0; j<6; j++)
    inc_data[j] = 2*CV_MAT_ELEM(*inc, float, j, 0);

  if(limityrot_)
    inc_data[4] = 0.0; // y-axis angle

  Vector<6> inc_vec(inc_data);
  SE3 M_inc = SE3::exp(inc_vec);

  Matrix<4> M_prop;
  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      M_prop[r][c] = (double)CV_MAT_ELEM(*states_prop_[i], float, r, c);

  // No IRLS
  Matrix<4> M_pred = M_prop;
  // Optimized with IRLS
  Matrix<4> M_opt = M_prop * M_inc;
 //  std::cout<<"The irls update is"<<std::endl;

  for(int r=0; r<4; r++)
  {
    for(int c=0; c<4; c++)
    {
      CV_MAT_ELEM(*states_pred_[i], float, r, c) = (float)M_pred[r][c];
      CV_MAT_ELEM(*states_opt_[i], float, r, c) = (float)M_opt[r][c];
      std::cout<<M_opt[r][c]<<" ";
    }
  }


  std::cout<<std::endl;
  // Calculate AR velocity
  CvMat* state_inv = cvCreateMat(4, 4, CV_32F);
  cvInvert(states_[i], state_inv, CV_SVD);
  cvMatMul(state_inv, states_pred_[i], ar_vel_[i]);
  cvMatMul(state_inv, states_opt_[i], ar_vel_opt_[i]);
  cvReleaseMat(&state_inv);

  cvReleaseMat(&W);
  cvReleaseMat(&Jt_W);
  cvReleaseMat(&Jt_W_J);
  cvReleaseMat(&Jt_W_J_inv);
  cvReleaseMat(&Jt_W_e);
  cvReleaseMat(&inc);*/
}

int CParticleFilter::GetBestParticle()
{
  int best_idx = -1;
  //float best_prob = 0.0;
  float best_prob = -1.0;
  for(int i=0; i<num_particle_; i++)
  {
    if(best_prob < prob_[i])
    {
      best_prob = prob_[i];
      best_idx = i;
    }
  }
  assert(best_idx >= 0 && best_idx < num_particle_);
  return best_idx;
}

bool CParticleFilter::Resample(float beta, bool bCompAR/*=false*/, bool bCompMean/*=false*/)
{
  // particle resampling
  float sum = 0.0f;

  for(int i=0; i<num_particle_; i++)       sum += prob_[i];
  // When opengl window is occluded, all of 'prob_' will be zero. In that case, we set uniform weight.
  if(sum == 0.0f)
  {
    neff_ = 0;
    return false; // invalid state -> re-initialize
  }
  else
    for(int i=0; i<num_particle_; i++)   weight_[i] = pow(prob_[i]/sum, beta); // normalize

  // normalize again
  sum = 0.0f;
  for(int i=0; i<num_particle_; i++)   sum += weight_[i];
  for(int i=0; i<num_particle_; i++)   weight_[i] /= sum;

  neff_ = 0.0f;
  for(int i=0; i<num_particle_; i++)
    neff_ += weight_[i]*weight_[i];
  neff_ = 1.0f/neff_;

  float *idx = new float[num_particle_];
  float *cumsum = new float[num_particle_+1];
  assert(num_particle_>0);
  idx[0] = (float)rand()/(float)RAND_MAX/(float)num_particle_;
  for(int i=1; i<num_particle_; i++)
    idx[i] = idx[i-1] + 1.0f/(float)num_particle_;
  cumsum[0] = 0.0f;
  for(int i=1; i<num_particle_+1; i++)
    cumsum[i] = cumsum[i-1] + weight_[i-1];

  int *outindex = new int[num_particle_];
  for(int i=0; i<num_particle_; i++)
  {
    outindex[i] = 0;
  }
  for(int i=0; i<num_particle_; i++)
  {
    for(int j=1; j<num_particle_+1; j++)
    {
      if(idx[i] > cumsum[j-1] && idx[i] <= cumsum[j])
      {
        outindex[i] = j-1;
        break;
      }
    }
  }

  // update resampled results to states
  for(int i=0; i<num_particle_; i++)
  {
    //cvCopy(states_pred_[outindex[i]], states_[i]); // Just predicted states based on AR states
    cvCopy(states_opt_[outindex[i]], states_[i]); // IRLS optimized states
    cvCopy(ar_vel_[outindex[i]], ar_vel_[i]);
    cvCopy(ar_vel_opt_[outindex[i]], ar_vel_opt_[i]);
    if(bCompAR) cvCopy(states_[i], states_prev_[i]); // save for later
  }

  // compute particle mean
  if(bCompMean)
  {
    calculateMeanState();
  }

  delete[] idx;
  delete[] cumsum;
  delete[] outindex;

  return true;
}

bool CParticleFilter::ResampleOpt(float beta, bool bCompAR/*=false*/, bool bCompMean/*=false*/)
{
  //  std::cout<<"here in resampling"<<std::endl;
 float sum = 0.0f;

  for(int i=0; i<num_particle_; i++)
  {
  //  sum += prob_[i];
    sum += prob_opt_[i];
  }
  // When opengl window is occluded, all of 'prob_' will be zero. In that case, we set uniform weight.
  if(sum == 0.0f)
  {
    neff_ = 0;
    return false; // invalid state -> re-initialize
  }
  else
  {
    for(int i=0; i<num_particle_; i++)
    {
  //    weight_[i] = pow(prob_[i]/sum, beta); // normalize
      weight_opt_[i] = pow(prob_opt_[i]/sum, beta); // normalize
    //  std::cout<<"weights"<<i<<" "<<prob_opt_[i]<<" "<<weight_opt_[i]<<std::endl;
    }
  }

  // normalize again
  sum = 0.0f;
  for(int i=0; i<num_particle_; i++)
  {
    //sum += weight_[i];
    sum += weight_opt_[i];
  }
  for(int i=0; i<num_particle_; i++)
  {
   // weight_[i] /= sum;
    weight_opt_[i] = (weight_opt_[i]/sum);
  }

  neff_ = 0.0f;
  for(int i=0; i<num_particle_; i++)
  {
   // neff_ += weight_[i]*weight_[i];
    neff_ += weight_opt_[i]*weight_opt_[i];
  }
  neff_ = 1.0f/neff_;

  float *idx = new float[num_particle_];
  float *cumsum = new float[num_particle_+1];
  assert(num_particle_>0);
  idx[0] = (float)rand()/(float)RAND_MAX/(float)(num_particle_);
  for(int i=1; i<num_particle_; i++)
    idx[i] = idx[i-1] + 1.0f/(float)(num_particle_);
  cumsum[0] = 0.0f;
  for(int i=1; i<num_particle_+1; i++)
  {
   // if(i<num_particle_+1)
  //    cumsum[i] = cumsum[i-1] + weight_[i-1];
  //  else
      cumsum[i] = cumsum[i-1] + weight_opt_[i-1];
  }

  int *outindex = new int[num_particle_];
  for(int i=0; i<num_particle_; i++)
  {
    outindex[i] = 0;
  }
  for(int i=0; i<num_particle_; i++)
  {
    for(int j=1; j<num_particle_+1; j++)
    {
      if(idx[i] > cumsum[j-1] && idx[i] <= cumsum[j])
      {
        outindex[i] = j-1;
        break;
      }
    }
  }

  // Update resampled results to states
  for(int i=0; i<num_particle_; i++)
  {
    //cvCopy(states_pred_[outindex[i]], states_[i]); // Just predicted states based on AR states
  //  std::cout<<outindex[i]<<" "<<i<<std::endl;
    cvCopy(states_opt_[outindex[i]], states_[i]); // IRLS optimized states
    cvCopy(ar_vel_[outindex[i]], ar_vel_[i]);
    cvCopy(ar_vel_opt_[outindex[i]], ar_vel_opt_[i]);
    if(bCompAR) cvCopy(states_[i], states_prev_[i]); // save for later
  }

  // compute particle mean
  if(bCompMean)
  {
    calculateMeanState();
  }

  delete[] idx;
  delete[] cumsum;
  delete[] outindex;

  return true;
}

void CParticleFilter::calculateWeights(int i, CvMat* e, vector<CObjectModel::SamplePoint> &vSamplePt, int maxD, float lamda_e/*=1.0f*/, float lamda_v/*=1.0f*/, bool update2optimized/*=false*/)
{


  //std::cout<<"number"<<i<<std::endl;

  if(e == NULL)
  {
    if(!update2optimized)
      prob_[i] = 0.0f;
    else
      prob_opt_[i] = 0.0f;
    return;
  }

  int NumOfVisibleSamplePoint = e->rows;

  double dist_max = maxD;
  vector<CvPoint2D32f> pts;
  CvPoint2D32f center = cvPoint2D32f(0, 0);
  int nov = 0;
  for(int k=0; k<int(vSamplePt.size()); k++)
  {
    if(vSamplePt[k].dist < dist_max) // only valid points
    {
      pts.push_back(vSamplePt[k].coord2);
      center.x += vSamplePt[k].coord2.x;
      center.y += vSamplePt[k].coord2.y;
      nov++;
    }
  }
  center.x /= nov;
  center.y /= nov;

 // std::cout<<"number"<<i<<std::endl;

  float eta = 0.0f;
  for(int k=0; k<pts.size(); k++)
  {
    eta += sqrt(pow(pts[k].x - center.x, 2) + pow(pts[k].y - center.y, 2));
  }
  eta_[i] = eta/nov;

  // calculate probability of each particle
  // get mean_e
  CvScalar mean_e = cvAvg(e);
  float dist1 = -(float)mean_e.val[0];
  float dist2 = -((float)(vSamplePt.size() - e->rows)/(float)vSamplePt.size());
 // dist2 = 0.0f;

  if(!update2optimized)
 {   prob_[i] = exp(lamda_e*dist1+lamda_v*dist2);
     prob_opt_[i] = exp(lamda_e*dist1+lamda_v*dist2);
//    std::cout<<"Weights "<<i<<" "<<prob_opt_[i]<<" "<<prob_[i]<<std::endl;
  }
  else
  {  prob_opt_[i] = exp(lamda_e*dist1+lamda_v*dist2);
  // std::cout<<"Weights "<<i<<" "<<prob_opt_[i]<<" "<<std::endl;
  }
 // std::cout<<prob_[i]<<" ";

}

void CParticleFilter::calculateMeanState(CvMat* MeanState/*=NULL*/)
{
  CvMat* meanR = cvCreateMat(3, 3, CV_32F);
  CvMat* meanT = cvCreateMat(3, 1, CV_32F);
  CvMat* _R = cvCreateMat(3, 3, CV_32F);
  CvMat* _T = cvCreateMat(3, 1, CV_32F);
  cvSetZero(meanR);
  cvSetZero(meanT);
  for(int i=0; i<num_particle_; i++)
  {
    cvAdd(meanR, cvGetSubRect(states_[i], _R, cvRect(0, 0, 3, 3)), meanR);
    cvAdd(meanT, cvGetSubRect(states_[i], _T, cvRect(3, 0, 1, 3)), meanT);
  }
  cvConvertScale(meanR, meanR, 1./(double)num_particle_);
  cvConvertScale(meanT, meanT, 1./(double)num_particle_);
  cvTranspose(meanR, meanR);

  CvMat* U = cvCreateMat(3, 3, CV_32F);
  CvMat* S = cvCreateMat(3, 3, CV_32F);
  CvMat* V = cvCreateMat(3, 3, CV_32F);
  double detR = cvDet(meanR);
  cvSVD(meanR, S, U, V, CV_SVD_MODIFY_A|CV_SVD_U_T);
  if(detR > 0.0)
    cvMatMul(V, U, meanR);
  else
  {
    CvMat* diagM = cvCreateMat(3, 3, CV_32F);
    cvSetIdentity(diagM);
    CV_MAT_ELEM(*diagM, float, 2, 2) = -1.0f;
    cvMatMul(V, diagM, diagM);
    cvMatMul(diagM, U, meanR);
    cvReleaseMat(&diagM);
  }

  if(MeanState) // update to parameter
  {
    cvGetSubRect(MeanState, _R, cvRect(0, 0, 3, 3));
    cvGetSubRect(MeanState, _T, cvRect(3, 0, 1, 3));
  }
  else // update to member variable
  {
    cvGetSubRect(mean_state_, _R, cvRect(0, 0, 3, 3));
    cvGetSubRect(mean_state_, _T, cvRect(3, 0, 1, 3));
  }
  cvCopy(meanR, _R);
  cvCopy(meanT, _T);

  cvReleaseMat(&meanR);
  cvReleaseMat(&meanT);
  cvReleaseMat(&_R);
  cvReleaseMat(&_T);
  cvReleaseMat(&U);
  cvReleaseMat(&S);
  cvReleaseMat(&V);
}

void CParticleFilter::CorrectWeights()
{
  //SE3 M_SE3;
  Matrix<6> InvSigma;
  for(int i=0; i<6; i++)
    //InvSigma(i, i) = 10; // sigma^2 = 0.1
    InvSigma(i, i) = 100; // sigma^2 = 0.01
  //InvSigma(i, i) = 10000; // sigma^2 = 0.0001
  //InvSigma(i, i) = 1000000; // sigma^2 = 0.000001

  float c_max = -100.0;
  float c_min = 100.0;
  for(int i=0; i<2*num_particle_; i++)
  {
    Matrix<4> P;
    for(int r=0; r<4; r++)
      for(int c=0; c<4; c++)
        if(i<num_particle_)
          P(r, c) = CV_MAT_ELEM(*states_pred_[i], float, r, c);
        else
          P(r, c) = CV_MAT_ELEM(*states_opt_[i-num_particle_], float, r, c);

    SE3 P3;
    P3 = P;
    Vector<6> p3 = SE3::ln(P3);
    float f = 0.0;
    for(int j=0; j<num_particle_; j++)
    {
      Matrix<4> X;
      for(int r=0; r<4; r++)
        for(int c=0; c<4; c++)
          X(r, c) = CV_MAT_ELEM(*states_pred_[j], float, r, c);
      SE3 X3;
      X3 = X;
      Vector<6> x3 = SE3::ln(X3);

      Vector<6> dx = p3 - x3;
      Matrix<1> dxT_InvSigma_dx = dx.as_row()*InvSigma*dx.as_col();
      f += exp(-0.5*dxT_InvSigma_dx(1,1));
    }

    float g = 0.0;
    for(int j=0; j<2*num_particle_; j++)
    {
      Matrix<4> X;
      for(int r=0; r<4; r++)
        for(int c=0; c<4; c++)
          if(j<num_particle_)
            X(r, c) = CV_MAT_ELEM(*states_pred_[j], float, r, c);
          else
            X(r, c) = CV_MAT_ELEM(*states_opt_[j-num_particle_], float, r, c);
      SE3 X3;
      X3 = X;
      Vector<6> x3 = SE3::ln(X3);

      Vector<6> dx = p3 - x3;
      Matrix<1> dxT_InvSigma_dx = dx.as_row()*InvSigma*dx.as_col();
      g += 0.5*exp(-0.5*dxT_InvSigma_dx(1,1));
    }
    float correct_term = f/g;
    if(i<num_particle_)
      prob_[i] *= correct_term;
    else
     {// prob_opt_[i-num_particle_] *= correct_term;

        prob_opt_[i-num_particle_] = prob_opt_[i-num_particle_]*correct_term;
      //  std::cout<<i-num_particle_<<" "<<prob_opt_[i-num_particle_]<<" ";
      }
    if(correct_term > c_max)    c_max = correct_term;
    if(correct_term < c_min)    c_min = correct_term;
  }
  if(verbose_)
  {
    cout << "c_min: " << c_min << "\t";
    cout << "c_max: " << c_max << endl;
  }
}
