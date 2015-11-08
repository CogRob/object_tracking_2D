#pragma once

#include "tracker_pf.h"

class TextureParticleFilterTracker : public ParticleFilterTracker
{
public:
  TextureParticleFilterTracker()
    : pe_surf_(NULL),
      init_keyframes_(false)
  {

  }

  virtual ~TextureParticleFilterTracker()
  {
    if(pe_surf_) delete pe_surf_;
  }

  virtual bool initTracker(std::string &obj_name, std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height, CvMat* pose_init, std::string ach_channel)
  {
    TrackerBase::initTracker(obj_name, cam_name, intrinsic, distortion, width, height, pose_init, ach_channel);

    initPoseEstimationSURF(width, height, obj_name, obj_name);

    cvCopy(pose_init_, pose_);

    for(int i=0;i<this->num_particles_; i++)
        pf_->Init(i,pose_init_);

    frame_num_after_init_ = 0;

    pf_->calculateMeanState();

    return (true);
  }

protected:
  CPoseEstimationSURF* pe_surf_;
  bool init_keyframes_;

  virtual bool initObjectModel(std::string name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker)
  {
    TrackerBase::initObjectModel(name, width, height, intrinsic, sample_step, maxd, dulledge, edge_tracker);

    std::string path = boost::filesystem::path(name).parent_path().string();

    obj_model_->loadKeyframes(path);
   // std::cout<<"adter the nit"<<path<<std::endl;
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


  bool initKeyframes()
  {
    initPoseEstimationSURF(width_, height_, std::string("data_")+obj_name_, obj_name_);

    return (true);
  }

  virtual void displayResults()
  {
    ParticleFilterTracker::displayResults();

    if(display_init_result_)
    {
      cvShowImage("Initialization", pe_surf_->getImage());
      display_init_result_ = false;
    }
  }

  virtual bool initialize()
  {
    TrackerBase::initialize();

    // (re-)initialize
    if(obj_model_->getNumOfKeyframes() > 0)
    {
      display_init_result_ = true;
      pe_surf_->setImage(img_gray_);

      std::cout<<"Reinitializing"<<std::endl;
      int num_corr = 0;
      CvMat* pose = pe_surf_->estimatePose(num_corr);
      std::cout<<num_corr<<std::endl;
      if(num_corr > min_keypoint_matches)
      {
        // 'pose' might be valid
        for(int i=0;i<this->num_particles_; i++)
          pf_->Init(i,pose);
     //   pf_->Init(0, pose);

        mutex_.lock();
        cvCopy(pose, pose_);
        mutex_.unlock();

        init_ = false;
        return true;
      }

    /*  if(pf_->GetNumOfParticle() == 1)
      {
        // init as 'IrlsTracker'
        int num_corr = 0;
        CvMat* pose = pe_surf_->estimatePose(num_corr);
        if(num_corr > 4)
        {
          // 'pose' might be valid
          pf_->Init(0, pose);
          
          mutex_.lock();
          cvCopy(pose, pose_);
          mutex_.unlock();
          
          init_ = false;
          return true;
        }
      }
     else // for more than 1 particle
      {
        int num_corr = 0;
        pe_surf_->PF_estimatePoses(num_corr, pf_->GetNumOfParticle(), pf_->GetStates());

        if(num_corr >= 9)
        {
          for(int i=0; i<(int)pf_->GetStates().size(); i++)
            pf_->Init(i, pf_->GetStates()[i]);

          pf_->calculateMeanState();

          mutex_.lock();
          cvCopy(pf_->GetMeanState(), pose_);
          mutex_.unlock();

          init_ = false;
          return true;
        }
      }*/
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
      cvCopy(pose_init_, pose_);
      for(int i=0;i<this->num_particles_; i++)
         pf_->Init(i,pose_init_);
      frame_num_after_init_ = 0;
      pf_->calculateMeanState();
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

      if(init_keyframes_)
        initKeyframes();
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

    /*  if(num_anneal_level == 1)
        pf_->Propagate(noise_l_, noise_h_, true);
      else
        pf_->Propagate(alpha_[l], alpha_[l], l == num_anneal_level-1 ? true : false);*/
      
      pf_->Propagate(noise_l_, noise_h_, true);
     // std::cout<<"weights";

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
      //  cvConvertScale(e,e,0.2);
        pf_->Update_IRLS(p, J, e, obj_model_->getNumberOfVisibleSamplePoints());
      //  std::cout<<"in particke filtering before"<<obj_model_->getNumberOfVisibleSamplePoints()<<std::endl;
        // calculate weights
        pf_->calculateWeights(p, e, obj_model_->getVisibleSamplePoints(), maxd_, lamda_e_, lamda_v_,false);
        // release after use them
        if(J) cvReleaseMat(&J);
        if(e) cvReleaseMat(&e);

#if 0 // disabled for performance (enable for valid pf)
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
#endif
      }

   //   std::cout<<std::endl;

      // correct weights caused from considering optimized states
      if(pf_->GetNumOfParticle() > 1)
        pf_->CorrectWeights();

      // resampling
      bool valid;
      if(pf_->GetNumOfParticle() > 1)
       valid = pf_->ResampleOpt(beta_[l], num_anneal_level == 1? true : false, true); // and calculate particle mean
      else
        valid = pf_->Resample(beta_[l], true /*num_anneal_level == 1? true : false*/, true); // and calculate particle mean

     //calculating variance
   /*   cvCopy(pf_->GetMeanState(), pose_);

      CvMat *J = NULL, *e = NULL;
      edge_tracker_->PF_getJacobianAndError(pf_->GetMeanState(), obj_model_->getVisibleSamplePoints(), &J, &e);

      pf_->Update(1, J, e, obj_model_->getNumberOfVisibleSamplePoints());
      cvCopy(pf_->GetVariance(), covariance_);
      cv::Mat poset = Mat(covariance_);
     // std::cout<<"in particke filtering after "<<poset<<std::endl;
      if(J) cvReleaseMat(&J);
      if(e) cvReleaseMat(&e);//*/


      if(valid) // && th_neff_ratio_*static_cast<float>(pf_->GetNumOfParticle()) < pf_->GetNeff())
      {
        mutex_.lock();
        cvCopy(pf_->GetMeanState(), pose_);
 /*       cvCopy(pf_->GetVariance(), covariance_);*/
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
   // if(num_anneal_level > 1 && !init_)
    //{
      for(int p=0; p<pf_->GetNumOfParticle(); p++)
        cvCopy(pf_->GetState(p), pf_->GetPrevState(p));
    //}
  }
};
