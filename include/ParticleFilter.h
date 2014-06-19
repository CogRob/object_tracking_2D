#pragma once

#include <opencv/cv.h> // Added for using 'CvMat'
#include <vector>
// TooN
#include <TooN/TooN.h>      // 
#include <TooN/so3.h>       // for special orthogonal group
#include <TooN/se3.h>       // for special Euclidean group
#include <TooN/wls.h>       // for weighted least square
#include <TooN/Cholesky.h>

#include "ObjectModel.h"

using namespace std;
using namespace TooN;

class CParticleFilter
{
public:
  CParticleFilter(int NumOfParticle = 1, float AR_param = 0.0, bool limitYRotation = false);
  ~CParticleFilter(void);

  void Init(CvMat* X = NULL);             // init states and member variables
  void Init(int i, CvMat* X = NULL);      // init a state and member variables
  void Propagate(float noiseRateLow, float noiseRateHigh, bool bCompAR = false);        // propagate particles based on AR process
  void Update(int i, CvMat* J, CvMat* e, int NumOfVisibleSamplePoint, float lamda = 1.0f);    // update
  void Update_IRLS(int i, CvMat* J, CvMat* e, int NumOfVisibleSamplePoint);    // update using IRLS
  void calculateWeights(int i, CvMat* e, vector<CObjectModel::SamplePoint> &vSamplePt, int maxD, float lamda_e=1.0f, float lamda_v=1.0f, bool update2optimized=false);
  void calculateEtas(bool updateProb = true, float lamda_eta = 1.0f)
  {
    float max = 0.0f;
    float min = std::numeric_limits<float>::max();
    for(int i=0; i<num_particle_; i++)
    {
      if(max < eta_[i])  max = eta_[i];
      if(min > eta_[i])  min = eta_[i];
    }
    for(int i=0; i<num_particle_; i++)
    {
      if(max-min <= 0.0)
        eta_[i] = 0.0f;
      else
        eta_[i] = (max - eta_[i])/(max-min);
      if(updateProb)
      {
        prob_[i] *= exp(-lamda_eta*eta_[i]);
      }
    }
  }
  bool Resample(float beta, bool bCompAR = false, bool bCompMean = false);      // resampling (and compute paricle mean)
  bool ResampleOpt(float beta, bool bCompAR = false, bool bCompMean = false);   // Resampling with optimized particles
  void calculateMeanState(CvMat* MeanState = NULL);
  int GetNumOfParticle()      { return num_particle_;  }
  CvMat* GetState(int i)      { return states_[i];       }
  vector<CvMat*>& GetStates() { return states_;          }
  CvMat* GetPrevState(int i)  { return states_prev_[i];  }
  CvMat* GetPropState(int i)  { return states_prop_[i];  }
  CvMat* GetPredState(int i)  { return states_pred_[i];  }
  CvMat* GetOptState(int i)   { return states_opt_[i];   }
  CvMat* GetMeanState()       { return mean_state_;       }
  float GetNeff()             { return neff_;            }
  int GetBestParticle();
  void CorrectWeights();

protected:
  int             num_particle_;
  CvMat*          mean_state_;      // mean of states
  vector<CvMat*>  states_;          // SE(3) states
  vector<CvMat*>  states_prev_;     // SE(3) states (previous) - used for calculating AR velocity
  vector<CvMat*>  states_prop_;     // SE(3) states (propagated)
  vector<CvMat*>  states_pred_;     // SE(3) states (predicted)
  vector<CvMat*>  states_opt_;      // SE(3) states (optimized)
  vector<CvMat*>  ar_vel_;          // Auto-regressive velocity
  vector<CvMat*>  ar_vel_opt_;      // Auto-regressive velocity for optimized particles
  vector<float>   weight_;          // weight of each particle
  vector<float>   weight_opt_;      // weight of each particle
  vector<float>   prob_;            // probability of each particle
  vector<float>   prob_opt_;        // probability of each particle
  vector<float>   eta_;             // penalizing term for covered area of sample points 
  float           ar_param_;        // Auto-regressive velocity parameter
  CvMat*          state_cov_;       // state covariance
  float           dt_;
  float           neff_;            // number of effective particles
  bool            limityrot_;
  bool            verbose_;
};
