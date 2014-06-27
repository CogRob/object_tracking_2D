#pragma once

#include "tracker_base.h"

class IrlsTracker : public TrackerBase
{
public:
  IrlsTracker()
    : pe_surf_(NULL)
    , init_keyframes_(false)
  {
    // init pose_ if you want
    cvSetIdentity(pose_);
    CV_MAT_ELEM(*pose_, float, 2, 3) = 1.0f; // 1.0 meter in front of camera
  }

  virtual ~IrlsTracker()
  {
    if(pe_surf_) delete pe_surf_;
  }

  virtual bool initTracker(std::string &obj_name, std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height, CvMat* pose_init, std::string ach_channel)
  {
    TrackerBase::initTracker(obj_name, cam_name, intrinsic, distortion, width, height, pose_init, ach_channel);

    initPoseEstimationSURF(width, height, std::string("data_")+obj_name, obj_name);

    return (true);
  }

protected:
  CPoseEstimationSURF* pe_surf_;
  bool init_keyframes_;

  virtual bool initObjectModel(std::string name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker)
  {
    TrackerBase::initObjectModel(name, width, height, intrinsic, sample_step, maxd, dulledge, edge_tracker);

    obj_model_->loadKeyframes(name);

    return (true);
  }

  bool initPoseEstimationSURF(int width, int height, std::string data_name, std::string &obj_name)
  {
    if(!pe_surf_)
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
    TrackerBase::displayResults();

    if(display_init_result_)
    {
      cvShowImage("Initialization", pe_surf_->getImage());
      display_init_result_ = false;
    }
  }

  virtual bool initialize()
  {

    //printf("starting initialize() in tracker_irls.h\n");

    TrackerBase::initialize();

    // (re-)initialize

    //printf("before for loop\n");
    //printf("obj_model_->getNumOfKeyframes(): %d\n", obj_model_->getNumOfKeyframes());

    if(obj_model_->getNumOfKeyframes() > 0)
    {
      display_init_result_ = true;
      pe_surf_->setImage(img_gray_);
      int num_corr = 0;      
           
      CvMat* pose = pe_surf_->estimatePose(num_corr);

      printf("# of matches...(%d)\n", num_corr);
      //printf("Threshold...(%d)\n", min_keypoint_matches);


      if(num_corr > min_keypoint_matches)
      {
        // 'pose' might be valid
        mutex_.lock();
        cvCopy(pose, pose_);
	sendPoseACH(pose_); //AKAN	
        mutex_.unlock();
        init_ = false;

        return true;
      }
    }

    return false;
  }

  bool initKeyframes()
  {
    initPoseEstimationSURF(width_, height_, std::string("data_")+obj_name_, obj_name_);

    return (true);
  }

  virtual void handleKey(char key)
  {
    switch(key)
    {
    case 'r':
    case 'R':
      if(!init_)
        init_ = true;
      // init pose_ if you want
      //cvSetIdentity(pose_);
      //CV_MAT_ELEM(*pose_, float, 2, 3) = 0.2f; // 1.0 meter in front of camera
      cvCopy(pose_init_, pose_);
      break;
    case 't':
    case 'T':
      if(init_)
        init_ = false;
      break;
    case 'k':
    case 'K':
      // Save keyframe into files - gray jpeg file and pose xml file
      obj_model_->saveKeyframe(obj_name_, img_gray_, pose_);
      init_keyframes_ = true;
      break;
    case 27:
      run_ = false;
      break;
    }
  }
  
  virtual void tracking()
  {
    // when a new keyfram is saved, init keyframe related sutff
    if(init_keyframes_)
      initKeyframes();

    // 'getEdge' returns Berkeley edge if it is available, otherwise returns NULL
    // 'extractEdge' extracts Canny edge if the fourth edge is NULL
    obj_model_->extractEdge(img_gray_, smooth_size_, th_canny_l_, th_canny_h_, cam_->getEdge());
    obj_model_->extractEdgeOri(img_gray_, smooth_size_);
    // update the initial pose to object model for displaying
    obj_model_->setModelviewMatrix(pose_);
    // draw object model with visibility test
    obj_model_->findVisibleSamplePoints();
    // find normal of each sampling point
    obj_model_->findNormalUsingEdgeCoord();
    // calculate error between sampling points and nearest edge
    obj_model_->findEdgeCorrespondences();
    
    if(obj_model_->isEnoughValidSamplePoints(th_valid_sample_points_ratio_))
    {
      edge_tracker_->getEstimatedPoseIRLS(edge_tracker_->getPose(), pose_, obj_model_->getVisibleSamplePoints());
      mutex_.lock();
      //cvCopy(pose_optimized, pose_);
      cvCopy(edge_tracker_->getPose(), pose_);
      mutex_.unlock();
    }
    else
    {
      // not enough valid sample points
      init_ = true; // (re-)init again
    }
  }
};
