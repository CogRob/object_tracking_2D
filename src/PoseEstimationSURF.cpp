
#include "PoseEstimationSURF.h"
#include "ObjectModel.h"
#include "epnp.h"

#include "Timer.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace cv;

CPoseEstimationSURF::CPoseEstimationSURF(int width, int height, std::string& img_path, CObjectModel* obj_model, CvMat* intrinsic_params, CvMat* distortion_params, std::string& objName, bool dispaly/*=true*/)
  : verbose_(false)
  , display_(dispaly)
  , draw_type_(CV_AA) // anti-aliasing drawing (CV_AA is 16), about 2ms more required
{
  ms_ = cvCreateMemStorage(0);
  // tested different hessian threshold (200, 500, 5000), but no significant performance change
  surf_params_ = cvSURFParams(500, 1);

  // allocate images
  img_input_ = cvCreateImage(cvSize(width, height), 8, 1);
  img_object_ = cvCreateImage(cvSize(width, height), 8, 1);
  obj_model_ = obj_model;

  // Init CvSeq data
  seq_keypoints_ = NULL;
  seq_descriptors_ = NULL;

  // create an image showing the result
  img_result_ = cvCreateImage(cvSize(width*2, height), 8, 3);

  // creat and init
  pose_ = cvCreateMat(4, 4, CV_32F);
  cvSetIdentity(pose_);

  intrinsic_ = intrinsic_params;
  distortion_ = distortion_params;

  // check that there is the file
  string templateFileName("data_" + objName + "/" + objName + ".txt");
  fstream file;
  file.open(templateFileName.c_str());
  if(file.is_open())
  {
    file.close();
    lf_.Configure("para_line_fitter.txt");
    lm_.Configure("para_line_matcher.txt");
    lf_.Init();
    lm_.Init(templateFileName.c_str());
  }

  // HACK:
  cv::initModule_nonfree(); // for SURF feature
}

CPoseEstimationSURF::~CPoseEstimationSURF(void)
{
  cvReleaseMat(&pose_);
}

void CPoseEstimationSURF::buildKdTree(vector<IplImage*>& keyframe, vector<CvMat*>& pose, vector<CvMat*>& keypoint2D, vector<CvMat*>& keypoint3D, vector<CvMat*>& descriptor)
{
  if(keyframe.size() == 0 || pose.size() == 0 || keypoint2D.size() == 0 || keypoint3D.size() == 0 || descriptor.size() == 0)
    return;

  assert(keyframe.size() == keypoint2D.size());
  assert(keypoint2D.size() == keypoint3D.size());
  assert(keypoint3D.size() == descriptor.size());

  keyframe_images_ = keyframe;

  // Save keyframe descriptor into CvMat
  int dims = descriptor[0]->cols;
  int row = 0;
  for(int i=0; i < keyframe.size(); i++)
  {
    row += keypoint2D[i]->rows;
  }

  cv::Mat kfd(row, dims, CV_32F);

  keyframe_lut_.resize(row);
  keyframe_keypoints_2d_.resize(row);
  keyframe_keypoints_3d_.resize(row);

  // Save keyframe descriptor into CvMat
  int k = 0;
  for(int h = 0; h < keyframe.size(); h++)
  {
    for(int i = 0; i < keypoint2D[h]->rows; i++ )
    {
      keyframe_keypoints_2d_[i+k] = CV_MAT_ELEM(*keypoint2D[h], CvPoint2D32f, i, 0);
      keyframe_keypoints_3d_[i+k] = CV_MAT_ELEM(*keypoint3D[h], CvPoint3D32f, i, 0);
      keyframe_lut_[i+k] = h;
      for(int j=0; j<dims; j++)
        kfd.at<float>(i+k, j) = CV_MAT_ELEM(*descriptor[h], float, i, j);
    }
    k += keypoint2D[h]->rows;
  }

  kfd_ = kfd;
}

CvMat* CPoseEstimationSURF::estimatePose(int &num_of_corr)
{
  // Using keyframes + EPnP RANSAC

  // Clear earlier storage and sequence
  if(seq_keypoints_)    cvClearSeq(seq_keypoints_);
  if(seq_descriptors_)  cvClearSeq(seq_descriptors_);
  cvClearMemStorage(ms_);

  // Extract SURF features on test images.
  cvExtractSURF(img_input_, 0, &seq_keypoints_, &seq_descriptors_, ms_, surf_params_);
  // Find the initial correspondence with Nearest Neighborhood
  findCorrespondenceNN_FLANN(seq_keypoints_, seq_descriptors_, corr_, keyframe_images_.size());
  // Refine the correspondences with RANSAC and estimate pose
  num_of_corr = refineCorrespondenceEpnpRANSAC(corr_, outliers_obj_2d_, outliers_obj_3d_, outliers_img_2d_, inliers_obj_2d_, inliers_obj_3d_, inliers_img_2d_, pose_);

  // Copy image and object images
  cvSetImageROI(img_result_, cvRect(0, 0, img_object_->width, img_object_->height));
  cvCvtColor(keyframe_images_[keyframe_idx_], img_result_, CV_GRAY2BGR );
  cvSetImageROI(img_result_, cvRect( img_object_->width, 0, img_input_->width, img_input_->height ) );
  cvCvtColor(img_input_, img_result_, CV_GRAY2BGR );
  cvResetImageROI(img_result_);

  // If the number of correspondence is smaller than 4, finish
  if(num_of_corr < 4)
  {
    printf("Insufficient matches...(%d)\n", num_of_corr);
    num_of_corr = 0;
    cvSetIdentity(pose_);
    return pose_;
  }

  // Display inliers/outliers
  for(int i=0; i<int(inliers_obj_2d_.size()); i++)
    cvLine(img_result_, cvPointFrom32f(inliers_obj_2d_[i]), 
    cvPoint(cvRound(inliers_img_2d_[i].x)+img_object_->width, cvRound(inliers_img_2d_[i].y)), CV_RGB(0,255,0), 1, draw_type_, 0);

  for(int i=0; i<int(outliers_obj_2d_.size()); i++)
    cvLine(img_result_, cvPointFrom32f(outliers_obj_2d_[i]), 
    cvPoint(cvRound(outliers_img_2d_[i].x) + img_object_->width, cvRound(outliers_img_2d_[i].y)), CV_RGB(255,100,100), 1, draw_type_, 0);

  // Draw object
  obj_model_->displayPoseLine(img_result_, pose_, CV_RGB(255, 255, 0), 1, true); // on the right side

  return pose_;
}

void CPoseEstimationSURF::PF_estimatePoses(int &num_of_corr, int numOfParticle, vector<CvMat*>& states)
{
  // Using keyframes + EPnP + random draw
  
  // Clear earlier storage and sequence
  if(seq_keypoints_)    cvClearSeq(seq_keypoints_);
  if(seq_descriptors_)  cvClearSeq(seq_descriptors_);
  cvClearMemStorage(ms_);

  // Extract SURF features on test images.
  cvExtractSURF(img_input_, 0, &seq_keypoints_, &seq_descriptors_, ms_, surf_params_);

  // Find the initial correspondence with Nearest Neighborhood
  findCorrespondenceNN_FLANN(seq_keypoints_, seq_descriptors_, corr_, keyframe_images_.size());

  int noc = int(corr_.size()/2); // noc: number of correspondence

  if(noc < 9)
  {
    num_of_corr = noc;
  }
  else
  {
    // estimate multiple poses using ePnP
    num_of_corr = PF_estimateMultiplePosesEpnp(corr_, outliers_obj_2d_, outliers_obj_3d_, outliers_img_2d_, inliers_obj_2d_, inliers_obj_3d_, inliers_img_2d_, states, numOfParticle);
  }

  // If the number of correspondence is smaller than 4, finish
  //if(num_of_corr <= 4)
  if(num_of_corr < 9)
  {
    printf("Insufficient matches...(%d)\n", num_of_corr);
    return;
  }

  if(display_)
  {
    // copy image and object images
    cvSetImageROI(img_result_, cvRect(0, 0, img_object_->width, img_object_->height));
    cvCvtColor(keyframe_images_[keyframe_idx_], img_result_, CV_GRAY2BGR );
    cvSetImageROI(img_result_, cvRect( img_object_->width, 0, img_input_->width, img_input_->height ) );
    cvCvtColor(img_input_, img_result_, CV_GRAY2BGR );
    cvResetImageROI(img_result_);

    inliers_obj_2d_.resize(noc);
    inliers_img_2d_.resize(noc);
    for(int i = 0; i < noc; i++)
    {
      inliers_obj_2d_[i] = keyframe_keypoints_2d_[corr_[i*2]];
      inliers_img_2d_[i] = input_keypoints_2d_[corr_[i*2+1]];
    }

    // display correspondence
    for(int i=0; i<noc; i++)
      cvLine(
      img_result_, 
      cvPointFrom32f(inliers_obj_2d_[i]), 
      cvPoint(cvRound(inliers_img_2d_[i].x) + img_object_->width, cvRound(inliers_img_2d_[i].y)), 
      CV_RGB(0,255,255), 
      1, 
      draw_type_, 
      0
      );
  }
  return;
}

int CPoseEstimationSURF::PF_estimatePosesFDCM(float maxThreshold, int numOfParticle, vector<CvMat*>& states, vector<LMDetWind> &detWind, int smoothSize/*=1*/, int cannyLow/*=20*/, int cannyHigh/*=40*/, IplImage* displayImage/*=NULL*/)
{
  Timer timer;
  timer.start();

  // Using edge templates + random draw
  IplImage *inputImage = cvCloneImage(img_input_);
  IplImage *edgeImage = cvCloneImage(inputImage);

  if(displayImage) cvCvtColor(inputImage, displayImage, CV_GRAY2RGB);

  if(smoothSize > 0)
    cvSmooth(inputImage, inputImage, CV_GAUSSIAN, smoothSize, smoothSize);

  cvCanny(inputImage, edgeImage, cannyLow, cannyHigh);
  cvReleaseImage(&inputImage);

  // Line Fitting
  lf_.FitLine(edgeImage);
  //lf_.FitLine_omp(edgeImage);

#if 0
  lf_.DisplayEdgeMap(edgeImage);
#endif

  // FDCM Matching
  lm_.MultiShapeDetectionWithVaryingTemplateSize(lf_, (double)maxThreshold, detWind);

  std::cout << detWind.size() << " detections..." << std::endl;

  for(size_t i=0; i<detWind.size(); i++)
  {
    std::cout << detWind[i].x_ << " " << detWind[i].y_ << " " << detWind[i].width_ << " " << detWind[i].height_ << " " << detWind[i].cost_ << " " << detWind[i].count_ << " " << detWind[i].scale_ << " " << detWind[i].aspect_ << " " << detWind[i].tidx_ << std::endl;
  }

  int maxDet = lm_.GetNumOfTemplates();

  if(detWind.size() > 0 && displayImage)
    for(size_t i=1; i<(detWind.size()<maxDet ? detWind.size() : maxDet); i++)
      DrawDetWind(displayImage, detWind[i].x_, detWind[i].y_, detWind[i].width_, detWind[i].height_, cvScalar(255,255,0), 1);

  if(detWind.size() > 0 && displayImage)
    DrawDetWind(displayImage, detWind[0].x_, detWind[0].y_, detWind[0].width_, detWind[0].height_, cvScalar(0,255,255), 1);

  if(edgeImage)   cvReleaseImage(&edgeImage);

  if(detWind.size() > 0)
  {
    // Calculate coarse pose
    float lamda = 1.0f;

    int numOfDet = detWind.size() < maxDet ? detWind.size() : maxDet;
    vector<CvMat*> poses;
    vector<float> vweight;
    poses.resize(numOfDet);
    vweight.resize(numOfDet);
    float u0 = CV_MAT_ELEM(*intrinsic_, float, 0, 2);
    float v0 = CV_MAT_ELEM(*intrinsic_, float, 1, 2);
    float fx = CV_MAT_ELEM(*intrinsic_, float, 0, 0);
    float fy = CV_MAT_ELEM(*intrinsic_, float, 1, 1);
    for(int i = 0; i<numOfDet; i++)
    {
      poses[i] = cvCreateMat(4, 4, CV_32F);
      cvCopy(obj_model_->getEdgeTemplatePose(static_cast<int>(detWind[i].tidx_)), poses[i]);
      int x = detWind[i].x_ + detWind[i].width_/2;
      int y = detWind[i].y_ + detWind[i].height_/2;
      float Z = CV_MAT_ELEM(*poses[i], float, 2, 3) = CV_MAT_ELEM(*poses[i], float, 2, 3)/detWind[i].scale_;
      float X = (float(x) - u0)*Z/fx;
      float Y = (float(y) - v0)*Z/fy;
      CV_MAT_ELEM(*poses[i], float, 0, 3) = X;
      CV_MAT_ELEM(*poses[i], float, 1, 3) = Y;

      vweight[i] = exp(-lamda*(float)detWind[i].cost_);
    }

    float *weight = new float[numOfParticle];
    for(int i = 0; i < numOfParticle; i++)
    {
      cvCopy(poses[i % numOfDet], states[i]);
      weight[i] = vweight[i % numOfDet];
    }

    // randomly draw from weight
    float sum = 0.0f;
    for(int i=0; i<numOfParticle; i++)   sum += weight[i];
    for(int i=0; i<numOfParticle; i++)   weight[i] = weight[i]/sum; // normalize

    float *idx = new float[numOfParticle];
    float *cumsum = new float[numOfParticle+1];
    assert(numOfParticle>0);
    idx[0] = (float)rand()/(float)RAND_MAX/(float)numOfParticle;
    for(int i=1; i<numOfParticle; i++)
      idx[i] = idx[i-1] + 1.0f/(float)numOfParticle;
    cumsum[0] = 0.0f;
    for(int i=1; i<numOfParticle+1; i++)
      cumsum[i] = cumsum[i-1] + weight[i-1];

    int *outindex = new int[numOfParticle];
    for(int i=0; i<numOfParticle; i++)
    {
      outindex[i] = 0;
    }
    for(int i=0; i<numOfParticle; i++)
    {
      for(int j=1; j<numOfParticle+1; j++)
      {
        if(idx[i] > cumsum[j-1] && idx[i] <= cumsum[j])
        {
          outindex[i] = j-1;
          break;
        }
      }
    }

    // update resampled results to states
    for(int i=0; i<numOfParticle; i++)
    {
      cvCopy(states[outindex[i]], states[i]);
    }

    delete[] idx;
    delete[] cumsum;
    delete[] outindex;
    delete[] weight;

    for(int i = 0; i < numOfDet; i++)
    {
      cvReleaseMat(&poses[i]);
    }
  }

  timer.printTimeMilliSec("PF_estimatePosesFDCM()");

  return detWind.size();
}

void CPoseEstimationSURF::findCorrespondenceNN_FLANN(const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs, int numOfKeyframes)
{
  if(imageDescriptors->total == 0)
  {
    ptpairs.clear();
    return;
  }

  CvSeqReader kreader, reader; 
  int dims = imageDescriptors->elem_size/sizeof(float);
  cv::Mat id(imageDescriptors->total, dims, CV_32F); // imageDescriptors
  input_keypoints_2d_.resize(imageDescriptors->total);

  // save image descriptor into CvMat
  cvStartReadSeq( imageKeypoints, &kreader );
  cvStartReadSeq( imageDescriptors, &reader );
  for(int i = 0; i < imageDescriptors->total; i++ )
  {
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* descriptor = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    input_keypoints_2d_[i] = (*kp).pt;
    for(int j=0; j<dims; j++)
      id.at<float>(i, j) = descriptor[j];
  }

  // find feature correspondences
  double th_ratio = 0.6;      // ratio test threshold
  cv::Mat indices(kfd_.rows, 2, CV_32S);
  cv::Mat dists(kfd_.rows, 2, CV_32F);

  cv::flann::Index flann_index(id, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
  flann_index.knnSearch(kfd_, indices, dists, 2, cv::flann::SearchParams(64)); // maximum number of leafs 

  ptpairs.clear();
  vector<int> idxpairs;

  // check results and save to 'ptpairs'
  int* indices_ptr = indices.ptr<int>(0);
  float* dists_ptr = dists.ptr<float>(0);
  for (int i=0; i<indices.rows; ++i) {
    if (dists_ptr[2*i] < th_ratio*dists_ptr[2*i+1]) {
      ptpairs.push_back(i); // image index
      ptpairs.push_back(indices_ptr[2*i]); // object index
      idxpairs.push_back(keyframe_lut_[indices_ptr[2*i]]);
    }
  }

  vector<int> cnt_buf;
  cnt_buf.resize(numOfKeyframes);

  for(int i=0; i<idxpairs.size(); i++)
  {
	  if(idxpairs[i]<cnt_buf.size() && idxpairs[i] > -1)
    cnt_buf[idxpairs[i]]++;
  }

  keyframe_idx_ = 0; // default
  int max_cnt = 0;
  for(int i=0; i<numOfKeyframes; i++)
  {
    if(max_cnt < cnt_buf[i])
    {
      max_cnt = cnt_buf[i];
      keyframe_idx_ = i;
    }
  }

  assert(keyframe_idx_ >= 0);
  printf("Matched keyframe: %d\n", keyframe_idx_);
  vector<int> ptpairs_new;
  for(int i=0; i<idxpairs.size(); i++)
  {
    if(idxpairs[i] == keyframe_idx_)
    {
      // check adding correspondence is already saved
      bool added = false;
      for(int j=0; j<int(ptpairs_new.size()/2); j++)
      {
        if(ptpairs_new[2*j] == ptpairs[2*i])
        {
          added = true;
          break;
        }
      }

      if(!added)
      {
        ptpairs_new.push_back(ptpairs[2*i]);
        ptpairs_new.push_back(ptpairs[2*i+1]);
      }
    }
  }

  ptpairs = ptpairs_new;
}

// EPnP version
int CPoseEstimationSURF::refineCorrespondenceEpnpRANSAC(const vector<int>& ptpairs, vector<CvPoint2D32f>& objOutliers, vector<CvPoint3D32f>& objOutliers3D, vector<CvPoint2D32f>& imgOutliers, vector<CvPoint2D32f>& objInliers, vector<CvPoint3D32f>& objInliers3D, vector<CvPoint2D32f>& imgInliers, CvMat* pmPose)
{
  const int NOM = 7; // number of model parameters

  int n;
  int iter = 0;
  int k = 100000;
  const int max_k = 1000;
  int best_noi = 0;
  const float th = 20;
  const double p = 0.99;

  n = int(ptpairs.size()/2);
  if( n < 8 ) // at least 8 points are needed to estimate fundamental matrix
    return -1;

  objOutliers.resize(n);
  imgOutliers.resize(n);
  objOutliers3D.resize(n);
  for(int i = 0; i < n; i++ )
  {
    objOutliers[i] = keyframe_keypoints_2d_[ptpairs[i*2]];
    imgOutliers[i] = input_keypoints_2d_[ptpairs[i*2+1]];
    objOutliers3D[i] = keyframe_keypoints_3d_[ptpairs[i*2]];
  }

  epnp ePnP;

  float fu = CV_MAT_ELEM(*intrinsic_, float, 0, 0);
  float fv = CV_MAT_ELEM(*intrinsic_, float, 1, 1);
  float uc = CV_MAT_ELEM(*intrinsic_, float, 0, 2);
  float vc = CV_MAT_ELEM(*intrinsic_, float, 1, 2);

  vector<int> inlier_idx;
  inlier_idx.resize(n);
  vector<int> best_inlier_idx;
  best_inlier_idx.resize(n);

  ePnP.set_internal_parameters(uc, vc, fu, fv);
  ePnP.set_maximum_number_of_correspondences(NOM);

  CvRNG rng = cvRNG(cvGetTickCount());
  int rand_idx[NOM];
  CvMat* P = cvCreateMat(3,4,CV_32F);
  CvMat* P2 = cvCreateMat(3,4,CV_32F);
  CvMat* x3d_h = cvCreateMat(4, n, CV_32F);
  CvMat* x2d_proj = cvCreateMat(3, n, CV_32F);

  for(int i=0; i<n; i++)
  {
    CV_MAT_ELEM(*x3d_h, float, 0, i) = objOutliers3D[i].x;
    CV_MAT_ELEM(*x3d_h, float, 1, i) = objOutliers3D[i].y;
    CV_MAT_ELEM(*x3d_h, float, 2, i) = objOutliers3D[i].z;
    CV_MAT_ELEM(*x3d_h, float, 3, i) = 1.0;
  }

  double R_est[3][3], T_est[3];

  while(iter < k && iter < max_k)
  {
    // sampling
    for(int i=0; i<NOM; i++)
    {
      int temp_idx= 0;
      bool found = true;
      while(found)
      {
        temp_idx = cvRandInt(&rng) % n;
        found = false;
        for(int j=0; j<i; j++)
        {
          if(rand_idx[j] == temp_idx)
            found = true;
        }
      }
      rand_idx[i] = temp_idx;
    }
    // model parameters fitted to rand_idx
    ePnP.reset_correspondences();
    for(int i=0; i<NOM; i++)
    {
      ePnP.add_correspondence(objOutliers3D[rand_idx[i]].x, objOutliers3D[rand_idx[i]].y, objOutliers3D[rand_idx[i]].z, imgOutliers[rand_idx[i]].x, imgOutliers[rand_idx[i]].y);
    }
    double err = ePnP.compute_pose(R_est, T_est);

    // project rest points into the image plane
    CV_MAT_ELEM(*P, float, 0, 0) = R_est[0][0];
    CV_MAT_ELEM(*P, float, 0, 1) = R_est[0][1];
    CV_MAT_ELEM(*P, float, 0, 2) = R_est[0][2];

    CV_MAT_ELEM(*P, float, 1, 0) = R_est[1][0];
    CV_MAT_ELEM(*P, float, 1, 1) = R_est[1][1];
    CV_MAT_ELEM(*P, float, 1, 2) = R_est[1][2];

    CV_MAT_ELEM(*P, float, 2, 0) = R_est[2][0];
    CV_MAT_ELEM(*P, float, 2, 1) = R_est[2][1];
    CV_MAT_ELEM(*P, float, 2, 2) = R_est[2][2];

    CV_MAT_ELEM(*P, float, 0, 3) = T_est[0];
    CV_MAT_ELEM(*P, float, 1, 3) = T_est[1];
    CV_MAT_ELEM(*P, float, 2, 3) = T_est[2];

    cvGEMM(intrinsic_, P, 1, NULL, 0, P2, 0);

    // x2d_proj = P * x3d_h
    cvGEMM(P2, x3d_h, 1, NULL, 0, x2d_proj, 0);


    for(int i=0; i<n; i++)
    {
      float u = CV_MAT_ELEM(*x2d_proj, float, 0, i);
      float v = CV_MAT_ELEM(*x2d_proj, float, 1, i);
      float w = CV_MAT_ELEM(*x2d_proj, float, 2, i);

      CV_MAT_ELEM(*x2d_proj, float, 0, i) = u/w;
      CV_MAT_ELEM(*x2d_proj, float, 1, i) = v/w;
      // save reprojection error to third rows
      CV_MAT_ELEM(*x2d_proj, float, 2, i) = sqrt((u/w - imgOutliers[i].x)*(u/w - imgOutliers[i].x) + (v/w - imgOutliers[i].y)*(v/w - imgOutliers[i].y));
    }

    // Count number of inliers
    int noi = 0;
    for(int i=0; i<n; i++) 
    {
      if(rand_idx[i] != i && CV_MAT_ELEM(*x2d_proj, float, 2, i)  < th)
      {
        inlier_idx[i] = 1;
        noi++;
      }
      else
        inlier_idx[i] = 0;
    }

    if(noi > best_noi) 
    {
      for(int i=0; i<NOM; i++)
        inlier_idx[rand_idx[i]] = 1;
      best_noi = noi;
      best_inlier_idx = inlier_idx;
      // Determine adaptive number of iteration
      double e = 1. - (double)best_noi/(double)n;
      k = (int)(log(1. - p)/log(1. - pow(1.-e, NOM)));
    }

    iter++;
    if(verbose_) printf("(%d/%d) iter: %d/%d\n", iter, k, best_noi, n);
  }

  if(best_noi > 0)
  {
    ePnP.set_maximum_number_of_correspondences(best_noi+NOM);
    ePnP.reset_correspondences();
    for(int i=0; i<n; i++)
    {
      if(best_inlier_idx[i])
        ePnP.add_correspondence(objOutliers3D[i].x, objOutliers3D[i].y, objOutliers3D[i].z, imgOutliers[i].x, imgOutliers[i].y);
    }

    double err = ePnP.compute_pose(R_est, T_est);

    CV_MAT_ELEM(*pmPose, float, 0, 0) = R_est[0][0];
    CV_MAT_ELEM(*pmPose, float, 1, 0) = R_est[1][0];
    CV_MAT_ELEM(*pmPose, float, 2, 0) = R_est[2][0];
    CV_MAT_ELEM(*pmPose, float, 3, 0) = 0.0;
    CV_MAT_ELEM(*pmPose, float, 0, 1) = R_est[0][1];
    CV_MAT_ELEM(*pmPose, float, 1, 1) = R_est[1][1];
    CV_MAT_ELEM(*pmPose, float, 2, 1) = R_est[2][1];
    CV_MAT_ELEM(*pmPose, float, 3, 1) = 0.0;
    CV_MAT_ELEM(*pmPose, float, 0, 2) = R_est[0][2];
    CV_MAT_ELEM(*pmPose, float, 1, 2) = R_est[1][2];
    CV_MAT_ELEM(*pmPose, float, 2, 2) = R_est[2][2];
    CV_MAT_ELEM(*pmPose, float, 3, 2) = 0.0;
    CV_MAT_ELEM(*pmPose, float, 0, 3) = T_est[0];
    CV_MAT_ELEM(*pmPose, float, 1, 3) = T_est[1];
    CV_MAT_ELEM(*pmPose, float, 2, 3) = T_est[2];
    CV_MAT_ELEM(*pmPose, float, 3, 3) = 1.0;
  }

  // Display estimated pose
#if 0
  cout << "Found pose:" << endl;
  ePnP.print_pose(R_est, T_est);
#endif

  // Refined points
  objInliers.clear();
  imgInliers.clear();
  objInliers3D.clear();
  vector<CvPoint2D32f> pt1_out, pt2_out;
  vector<CvPoint3D32f> pt3_out;
  for(int i=0; i<n; i++)
  {
    if(best_inlier_idx[i] == 1) // inliers only
    {
      objInliers.push_back(objOutliers[i]);
      imgInliers.push_back(imgOutliers[i]);
      objInliers3D.push_back(objOutliers3D[i]);
    }
    else // outliers
    {
      pt1_out.push_back(objOutliers[i]);
      pt2_out.push_back(imgOutliers[i]);
      pt3_out.push_back(objOutliers3D[i]);
    }
  }

  objOutliers = pt1_out;
  imgOutliers = pt2_out;
  objOutliers3D = pt3_out;

  cvReleaseMat(&P);
  cvReleaseMat(&P2);
  cvReleaseMat(&x3d_h);
  cvReleaseMat(&x2d_proj);

  return int(objInliers.size());
}

// EPnP for PF version
int CPoseEstimationSURF::PF_estimateMultiplePosesEpnp(const vector<int>& ptpairs, vector<CvPoint2D32f>& objOutliers, vector<CvPoint3D32f>& objOutliers3D, vector<CvPoint2D32f>& imgOutliers, vector<CvPoint2D32f>& objInliers, vector<CvPoint3D32f>& objInliers3D, vector<CvPoint2D32f>& imgInliers, vector<CvMat*>& states, int numOfParticle)
{
  // For accurate estimation 'NOM' should be larger than '6'
  const int NOM = 7; // number of model parameters

  int noc;
  int best_noi = 0;
  const float th = 20;
  const double p = 0.99;

  noc = int(ptpairs.size()/2);
  if( noc < NOM )
    return -1;

  objOutliers.resize(noc);
  imgOutliers.resize(noc);
  objOutliers3D.resize(noc);
  for(int i = 0; i < noc; i++ )
  {
    objOutliers[i] = keyframe_keypoints_2d_[ptpairs[i*2]];
    imgOutliers[i] = input_keypoints_2d_[ptpairs[i*2+1]];
    objOutliers3D[i] = keyframe_keypoints_3d_[ptpairs[i*2]];
    //cout << objOutliers3D[i].x << " " << objOutliers3D[i].y << " " << objOutliers3D[i].z << endl;
  }

  epnp ePnP;

  float fu = CV_MAT_ELEM(*intrinsic_, float, 0, 0);
  float fv = CV_MAT_ELEM(*intrinsic_, float, 1, 1);
  float uc = CV_MAT_ELEM(*intrinsic_, float, 0, 2);
  float vc = CV_MAT_ELEM(*intrinsic_, float, 1, 2);

  vector<int> inlier_idx;
  inlier_idx.resize(noc);
  vector<int> best_inlier_idx;
  best_inlier_idx.resize(noc);

  ePnP.set_internal_parameters(uc, vc, fu, fv);
  ePnP.set_maximum_number_of_correspondences(NOM);

  CvRNG rng = cvRNG(cvGetTickCount());
  int rand_idx[NOM];
  CvMat* P = cvCreateMat(3,4,CV_32F);
  CvMat* P2 = cvCreateMat(3,4,CV_32F);
  CvMat* x3d_h = cvCreateMat(4, noc, CV_32F);
  CvMat* x2d_proj = cvCreateMat(3, noc, CV_32F);

  for(int i=0; i<noc; i++)
  {
    CV_MAT_ELEM(*x3d_h, float, 0, i) = objOutliers3D[i].x;
    CV_MAT_ELEM(*x3d_h, float, 1, i) = objOutliers3D[i].y;
    CV_MAT_ELEM(*x3d_h, float, 2, i) = objOutliers3D[i].z;
    CV_MAT_ELEM(*x3d_h, float, 3, i) = 1.0;
  }

  double R_est[3][3], T_est[3];

  float *weight = new float[numOfParticle];

  for(int par=0; par<numOfParticle; par++)
  {
    // sampling
    for(int i=0; i<NOM; i++)
    {
      int temp_idx= 0;
      bool found = true;
      while(found)
      {
        temp_idx = cvRandInt(&rng) % noc;
        found = false;
        for(int j=0; j<i; j++)
        {
          if(rand_idx[j] == temp_idx)
            found = true;
        }
      }
      rand_idx[i] = temp_idx;
    }
    // model parameters fitted to rand_idx
    ePnP.reset_correspondences();
    for(int i=0; i<NOM; i++)
    {
      ePnP.add_correspondence(objOutliers3D[rand_idx[i]].x, objOutliers3D[rand_idx[i]].y, objOutliers3D[rand_idx[i]].z, imgOutliers[rand_idx[i]].x, imgOutliers[rand_idx[i]].y);
    }
    double err = ePnP.compute_pose_gn(R_est, T_est);


    // project rest points into the image plane
    CV_MAT_ELEM(*P, float, 0, 0) = R_est[0][0];
    CV_MAT_ELEM(*P, float, 0, 1) = R_est[0][1];
    CV_MAT_ELEM(*P, float, 0, 2) = R_est[0][2];

    CV_MAT_ELEM(*P, float, 1, 0) = R_est[1][0];
    CV_MAT_ELEM(*P, float, 1, 1) = R_est[1][1];
    CV_MAT_ELEM(*P, float, 1, 2) = R_est[1][2];

    CV_MAT_ELEM(*P, float, 2, 0) = R_est[2][0];
    CV_MAT_ELEM(*P, float, 2, 1) = R_est[2][1];
    CV_MAT_ELEM(*P, float, 2, 2) = R_est[2][2];

    CV_MAT_ELEM(*P, float, 0, 3) = T_est[0];
    CV_MAT_ELEM(*P, float, 1, 3) = T_est[1];
    CV_MAT_ELEM(*P, float, 2, 3) = T_est[2];

    cvGEMM(intrinsic_, P, 1, NULL, 0, P2, 0);

    // >> x2d_proj = P * x3d_h
    cvGEMM(P2, x3d_h, 1, NULL, 0, x2d_proj, 0);


    for(int i=0; i<noc; i++)
    {
      float u = CV_MAT_ELEM(*x2d_proj, float, 0, i);
      float v = CV_MAT_ELEM(*x2d_proj, float, 1, i);
      float w = CV_MAT_ELEM(*x2d_proj, float, 2, i);

      CV_MAT_ELEM(*x2d_proj, float, 0, i) = u/w;
      CV_MAT_ELEM(*x2d_proj, float, 1, i) = v/w;
      // save reprojection error to third rows
      CV_MAT_ELEM(*x2d_proj, float, 2, i) = (u/w - imgOutliers[i].x)*(u/w - imgOutliers[i].x) + (v/w - imgOutliers[i].y)*(v/w - imgOutliers[i].y);
    }

    // count number of inliers
    int noi = 0;
    for(int i=0; i<noc; i++) 
    {
      if(rand_idx[i] != i && CV_MAT_ELEM(*x2d_proj, float, 2, i)  < th*th)
      {
        inlier_idx[i] = 1;
        noi++;
      }
      else
        inlier_idx[i] = 0;
    }

    CV_MAT_ELEM(*states[par], float, 0, 0) = R_est[0][0];
    CV_MAT_ELEM(*states[par], float, 1, 0) = R_est[1][0];
    CV_MAT_ELEM(*states[par], float, 2, 0) = R_est[2][0];
    CV_MAT_ELEM(*states[par], float, 3, 0) = 0.0;
    CV_MAT_ELEM(*states[par], float, 0, 1) = R_est[0][1];
    CV_MAT_ELEM(*states[par], float, 1, 1) = R_est[1][1];
    CV_MAT_ELEM(*states[par], float, 2, 1) = R_est[2][1];
    CV_MAT_ELEM(*states[par], float, 3, 1) = 0.0;
    CV_MAT_ELEM(*states[par], float, 0, 2) = R_est[0][2];
    CV_MAT_ELEM(*states[par], float, 1, 2) = R_est[1][2];
    CV_MAT_ELEM(*states[par], float, 2, 2) = R_est[2][2];
    CV_MAT_ELEM(*states[par], float, 3, 2) = 0.0;
    CV_MAT_ELEM(*states[par], float, 0, 3) = T_est[0];
    CV_MAT_ELEM(*states[par], float, 1, 3) = T_est[1];
    CV_MAT_ELEM(*states[par], float, 2, 3) = T_est[2];
    CV_MAT_ELEM(*states[par], float, 3, 3) = 1.0;

    float lamda = 3.0f;
    weight[par] = exp(-lamda*(float)(noc-NOM-noi)/float(noc-NOM));
  }

  // randomly draw from weight
  float sum = 0.0f;
  for(int i=0; i<numOfParticle; i++)   sum += weight[i];
  for(int i=0; i<numOfParticle; i++)   weight[i] = weight[i]/sum; // normalize

  float *idx = new float[numOfParticle];
  float *cumsum = new float[numOfParticle+1];
  assert(numOfParticle>0);
  idx[0] = (float)rand()/(float)RAND_MAX/(float)numOfParticle;
  for(int i=1; i<numOfParticle; i++)
    idx[i] = idx[i-1] + 1.0f/(float)numOfParticle;
  cumsum[0] = 0.0f;
  for(int i=1; i<numOfParticle+1; i++)
    cumsum[i] = cumsum[i-1] + weight[i-1];

  int *outindex = new int[numOfParticle];
  for(int i=0; i<numOfParticle; i++)
  {
    outindex[i] = 0;
  }
  for(int i=0; i<numOfParticle; i++)
  {
    for(int j=1; j<numOfParticle+1; j++)
    {
      if(idx[i] > cumsum[j-1] && idx[i] <= cumsum[j])
      {
        outindex[i] = j-1;
        break;
      }
    }
  }

  // update resampled results to states
  for(int i=0; i<numOfParticle; i++)
  {
    cvCopy(states[outindex[i]], states[i]);
  }

  delete[] idx;
  delete[] cumsum;
  delete[] outindex;
  delete[] weight;

  cvReleaseMat(&P);
  cvReleaseMat(&P2);
  cvReleaseMat(&x3d_h);
  cvReleaseMat(&x2d_proj);

  return noc;
}

void CPoseEstimationSURF::setImage(IplImage* img)
{
  // Update test image
  cvCopy(img, img_input_);
}





