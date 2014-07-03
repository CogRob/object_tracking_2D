#pragma once

#include "tracker_base.h"
#include "ParticleFilter.h"

class ParticleFilterTracker : public TrackerBase
{
public:
  ParticleFilterTracker()
    : ar_param_(0.0f)
    , num_particles_(100)
    , noise_l_(0.0005f)
    , noise_h_(0.01f)
    , num_annealing_layers_(1)
    , alpha_rate_(0.5f)
    , beta_rate_(0.5f)
    , noise_an_(0.05f)
    , th_ransac_(0.f)
    , th_ransac_iter_(1000)
    , lamda_e_(0.5f)
    , lamda_v_(25.f)
    , pf_(NULL)
    , th_neff_ratio_(0.2f)
  {
    mean_ = cvCreateMat(4, 4, CV_32F);
    
    initAnnealing();
  }

  virtual ~ParticleFilterTracker()
  {
    cvReleaseMat(&mean_);
    delete pf_;
  }

  bool initParticleFilter()
  {
    if(pf_) delete pf_;

    pf_ = new CParticleFilter (num_particles_, ar_param_, limityrot_);

    return (true);
  }

  inline void setNumAnnealingLayers(int l) { num_annealing_layers_= l; initAnnealing(); };
  inline void setNumParticle(int n) { num_particles_ = n; };
  inline float getThresholdRatioNeff() { return th_neff_ratio_; }
  inline void setThresholdRatioNeff(float t) { th_neff_ratio_ = t; }

protected:
  void initAnnealing()
  {
    alpha_.resize(num_annealing_layers_);
    beta_.resize(num_annealing_layers_);

    for(int i = 0; i < num_annealing_layers_; i++)
    {
      alpha_[i] = noise_an_ * pow(alpha_rate_, static_cast<float>(num_annealing_layers_ - 1 - i));
      beta_[i] = pow(beta_rate_, static_cast<float>(i));
    }
  }

  virtual void displayResults()
  {
    // draw particles
    for(int i = 0; i < pf_->GetNumOfParticle(); i++)
    {
      if(dulledge_) // for valid display, occlusion reasioning is required
      {
        obj_model_->setModelviewMatrix(pf_->GetPropState(i));
        obj_model_->findVisibleSamplePoints();
      }
      obj_model_->displayPoseLine(img_result_, pf_->GetPropState(i), CV_RGB(0, 255, 0), 1, false);
    }

    obj_model_->displaySamplePointsAndErrors(img_edge_); // display data of the last particle

    // draw mean particle
    if(dulledge_) // for valid display, occlusion reasioning is required
    {
      obj_model_->setModelviewMatrix(pf_->GetMeanState());
      obj_model_->findVisibleSamplePoints();
    }
    obj_model_->displayPoseLine(img_result_, pf_->GetMeanState(), CV_RGB(255, 255, 0), 2, false);
        
    cvShowImage("Result", img_result_);
    cvShowImage("Edge", img_edge_);
  }

  float ar_param_;
  int num_particles_;
  float noise_l_;
  float noise_h_;
  CvMat* mean_;
  CParticleFilter* pf_;
  // annealing process
  int num_annealing_layers_;
  float alpha_rate_;
  float beta_rate_;
  float noise_an_;
  std::vector<float> alpha_;
  std::vector<float> beta_;
  float th_ransac_;
  int th_ransac_iter_;
  float lamda_e_;
  float lamda_v_;
  float th_neff_ratio_;
};
