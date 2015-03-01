#pragma once

#include "tracker_pf.h"

class TexturelessParticleFilterTracker : public ParticleFilterTracker
{
public:
  TexturelessParticleFilterTracker()
    : pe_surf_(NULL)
    , th_cm_(0.2f)
  {

  }

  virtual ~TexturelessParticleFilterTracker()
  {
    if(pe_surf_) delete pe_surf_;
  }

  virtual bool initTracker(std::string &obj_name, std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height, CvMat* pose_init, std::string ach_channel)
  {
    TrackerBase::initTracker(obj_name, cam_name, intrinsic, distortion, width, height, pose_init, ach_channel);
    std::cout<<"it ttracker"<<std::endl;
    initPoseEstimationSURF(width, height, obj_name, obj_name);

    pf_->Init(pose_init);

    return (true);
  }

  inline void setThresholdCM(float th) { th_cm_ = th; };
  inline float getThresholdCM() { return th_cm_; };

protected:
  CPoseEstimationSURF* pe_surf_;
  float th_cm_;

  virtual bool initObjectModel(std::string name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker)
  {
    TrackerBase::initObjectModel(name, width, height, intrinsic, sample_step, maxd, dulledge, edge_tracker);

    obj_model_->loadKeyframes(name);

    return (true);
  }

  bool initPoseEstimationSURF(int width, int height, std::string data_name, std::string &obj_name)
  {
    if(pe_surf_) delete pe_surf_;
    
    pe_surf_ = new CPoseEstimationSURF(
      width, 
      height, 
      data_name, 
      obj_model_, 
      cam_->getIntrinsicParams(), 
      cam_->getDistortionParams(), 
      obj_name
    );

    pe_surf_->buildKdTree(
      obj_model_->getKeyframeImages(), 
      obj_model_->getKeyframePoses(), 
      obj_model_->getKeyframeKeypoints2D(), 
      obj_model_->getKeyframeKeypoints3D(), 
      obj_model_->getKeyframeDescriptors()
    );
    return (true);
  }

  virtual void displayResults()
  {
    ParticleFilterTracker::displayResults();

    if(display_init_result_)
    {
      cvShowImage("Initialization", pe_surf_->getImage());
      cvWaitKey(1500);
      display_init_result_ = false;
    }
  }

  virtual bool initialize()
  {
    TrackerBase::initialize();

    // (re-)initialize
    if(obj_model_->getNumOfEdgeTemplates() > 0)
    {
      display_init_result_ = true;
      pe_surf_->setImage(img_gray_);

      // estimate a set of poses based on random corresondences
      std::vector<LMDetWind> detWind;
      int numof_det = pe_surf_->PF_estimatePosesFDCM(th_cm_, pf_->GetNumOfParticle(), pf_->GetStates(), detWind, smooth_size_, th_canny_l_, th_canny_h_);
      
      if(numof_det > 0)
      {
        for(int i=0; i<(int)pf_->GetStates().size(); i++)
          pf_->Init(i, pf_->GetStates()[i]);

        pf_->calculateMeanState();

        mutex_.lock();
        cvCopy(pf_->GetMeanState(), pose_);

	

	printPose(pose_); //AKAN
	CV_MAT_ELEM(*pose_,float,2,3) = 1.20;
	
	//	pose_[2][3] = 1.2;

	sendPoseACH(pose_); //AKAN
        mutex_.unlock();

        init_ = false;
        return true;
      }
    }
    return false;
  }

  virtual void handleKey(char key)
  {
    switch(key)
    {
    case 'r':
    case 'R':
      if(!init_)
        init_ = true;
      break;
    case 't':
    case 'T':
      if(init_)
        init_ = false;
      break;
    case 27:
      run_ = false;
      break;
    }
  }
  
  virtual void tracking()
  {
    // do annealing process only after (re-)initialization
    int num_anneal_level = frame_num_after_init_ == 0 ? num_annealing_layers_ : 1;

    for(int l = num_anneal_level-1; l >= 0; l--)
    {
      // 'getEdge' returns Berkeley edge if it is available, otherwise returns NULL
      // 'extractEdge' extracts Canny edge if the fourth edge is NULL
      obj_model_->extractEdge(img_gray_, smooth_size_, th_canny_l_, th_canny_h_, cam_->getEdge());
      obj_model_->extractEdgeOri(img_gray_, smooth_size_);

      // reset previous drawn image
      if(display_)
        cvCvtColor(img_gray_, img_result_, CV_GRAY2BGR); // shoud be changed in better way

      if(num_anneal_level == 1)
        pf_->Propagate(noise_l_, noise_h_, true);
      else
        pf_->Propagate(alpha_[l], alpha_[l], l == num_anneal_level-1 ? true : false);
      
      for(int p = 0; p < pf_->GetNumOfParticle(); p++)
      {
        // update the initial pose to object model for displaying
        obj_model_->setModelviewMatrix(pf_->GetPropState(p));
        // draw object model with visibility test
        obj_model_->findVisibleSamplePoints();
        // find normal of each sampling point
        obj_model_->findNormalUsingEdgeCoord();
        // calculate error between sampling points and nearest edge
        obj_model_->findEdgeCorrespondences();

        if(th_ransac_ > 0.0f)
          obj_model_->refineEdgeCorrespondences_RANSAC(pf_->GetPropState(p), th_ransac_iter_, th_ransac_);

        // consider edge sample points only
        CvMat *J = NULL, *e = NULL;
        edge_tracker_->PF_getJacobianAndError(pf_->GetPropState(p), obj_model_->getVisibleSamplePoints(), &J, &e);
        pf_->Update_IRLS(p, J, e, obj_model_->getNumberOfVisibleSamplePoints());
        // calculate weights
        pf_->calculateWeights(p, e, obj_model_->getVisibleSamplePoints(), maxd_, lamda_e_, lamda_v_);
        // release after use them
        if(J) cvReleaseMat(&J);
        if(e) cvReleaseMat(&e);

        // calculate weight (likelihood now) for optimized particles
        if(pf_->GetNumOfParticle() > 1)
        {
          // update the initial pose to object model for displaying
          obj_model_->setModelviewMatrix(pf_->GetOptState(p));
          // draw object model with visibility test
          obj_model_->findVisibleSamplePoints();
          // find normal of each sampling point
          obj_model_->findNormalUsingEdgeCoord();
          // calculate error between sampling points and nearest edge
          obj_model_->findEdgeCorrespondences();

          if(th_ransac_ > 0.0f)
            obj_model_->refineEdgeCorrespondences_RANSAC(pf_->GetOptState(p), th_ransac_iter_, th_ransac_);
          e = NULL;
          edge_tracker_->PF_getError(pf_->GetOptState(p), obj_model_->getVisibleSamplePoints(), &e);
          // Calculate weights for optimized particles
          pf_->calculateWeights(p, e, obj_model_->getVisibleSamplePoints(), maxd_, lamda_e_, lamda_v_, true);
          if(e) cvReleaseMat(&e);
        }
      }

      // correct weights caused from considering optimized states
      if(pf_->GetNumOfParticle() > 1)
        pf_->CorrectWeights();

      // resampling
      bool valid;
      if(pf_->GetNumOfParticle() > 1)
        valid = pf_->ResampleOpt(beta_[l], num_anneal_level == 1? true : false, true); // and calculate particle mean
      else
        valid = pf_->Resample(beta_[l], num_anneal_level == 1? true : false, true); // and calculate particle mean

      if(valid) // && th_neff_ratio_*static_cast<float>(pf_->GetNumOfParticle()) < pf_->GetNeff())
      {
        mutex_.lock();
        cvCopy(pf_->GetMeanState(), pose_);

	

        mutex_.unlock();
      }
      else
      {
        // particle filter results is not valid
        init_ = true; // (re-)init again
        break; // stop annealing process
      }
    }

    // reset 'previous state of particles' to 'current state of particles' right after annealing
    if(num_anneal_level > 1 && !init_)
    {
      for(int p=0; p<pf_->GetNumOfParticle(); p++)
        cvCopy(pf_->GetState(p), pf_->GetPrevState(p));
    }
  }
};
