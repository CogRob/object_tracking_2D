#pragma warning( once : 4305 )
#pragma warning( once : 4244 )
#pragma warning( once : 4267 )

#include "object_tracking_2D/ObjectModel.h"
#include <GL/glu.h>
#include "object_tracking_2D/EdgeTracker.h"
#include <algorithm> // sort()
#include "object_tracking_2D/PoseEstimationSURF.h"
#include <limits>
#include <omp.h> // openmp
#include "object_tracking_2D/Timer.h"

CObjectModel::CObjectModel(string obj_name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker)
  : width_(width)
  , height_(height)
  , sample_step_(sample_step)
  , maxd_(maxd)
  , dulledge_(dulledge)
  , edge_tracker_(edge_tracker)
  , use_fine_orientation_(true)
  , draw_type_(CV_AA) // anti-aliasing drawing (CV_AA is 16), about 2ms more required
  , framebuffer_(0)
  , renderbuffer_(0)
  , depthbuffer_(0)
  , th_sharp_(0.9f)
  , include_starting_point_(true)
  , dl_(0)
{
  img_edge_ = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 1);
  img_gx_   = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_32F, 1);
  img_gy_   = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_32F, 1);

  initOpenGL(width_, height_);
  initFrameBufferObject(width_, height_);

  pose_ = cvCreateMat(4, 4, CV_32F);
  intrinsic_ = cvCreateMat(3, 3, CV_32F);
  // set the projection matrix for opengl with the camera intrinsic parameters
  setProjectionMatrix(intrinsic);

  meshmodel_ = NULL;
  num_keyframes_ = 0;
  loadObjectCADModel(obj_name);
  loadKeyframes(obj_name);

  num_edge_templates_ = 0;
  loadEdgeTemplates(obj_name);
}

CObjectModel::~CObjectModel(void)
{
  cvReleaseMat(&pose_);
  cvReleaseMat(&intrinsic_);

  glmDelete(meshmodel_);

  for(size_t i=0; i<keyframe_images_.size(); i++)
    cvReleaseImage(&keyframe_images_[i]);

  for(size_t i=0; i<keyframe_poses_.size(); i++)
    if(keyframe_poses_[i])
      cvReleaseMat(&keyframe_poses_[i]);

  for(size_t i=0; i<edge_template_poses_.size(); i++)
    if(edge_template_poses_[i])
      cvReleaseMat(&edge_template_poses_[i]);

  cvReleaseImage(&img_edge_);
  cvReleaseImage(&img_gx_);
  cvReleaseImage(&img_gy_);

  if(dl_)
    glDeleteLists(dl_, 1);

  // delete frame buffer object sutff
  if(framebuffer_)
  {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    // delete the renderbuffer attachments
    if(renderbuffer_) glDeleteRenderbuffers(1, &renderbuffer_);
    if(depthbuffer_)  glDeleteRenderbuffers(1, &depthbuffer_);
    // delete the framebuffer attachment
    glDeleteFramebuffers(1, &framebuffer_);
  }
}

bool CObjectModel::initOpenGL(int width, int height)
{
  // init GLUT
  int argc = 0;
  char** argv = NULL;
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(width, height);
  glutCreateWindow("OpenGL window");
  glutHideWindow();

  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_FLAT);

  // init GLEW
  GLenum err = glewInit();
  if(GLEW_OK != err)
  {
    // glewInit failed, something is seriously wrong.
    fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    exit (1);
  }
  fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));

  GLint bits;
  if(GLEW_ARB_occlusion_query)
  {
    glGetQueryivARB(GL_SAMPLES_PASSED_ARB, GL_QUERY_COUNTER_BITS_ARB, &bits);
    if (!bits) {
      printf("GL_QUERY_COUNTER_BITS_ARB is zero!\n");
      exit(-1);
    }
  }

  glGetIntegerv(GL_DEPTH_BITS, &bits);
  printf("Depthbits: %d\n", bits);

  if(GLEW_ARB_occlusion_query)
  {
    GLuint query;
    glGenQueriesARB(1, &query);
    assert(query > 0);
  }

  return (true);
}

bool CObjectModel::initFrameBufferObject(int width, int height)
{
  // check if FBO is supported by your video card
  bool fbo_supported = false;
  if(GLEW_ARB_framebuffer_object)
  {
    fbo_supported = true;
    std::cout << "Video card supports GL_ARB_framebuffer_object." << std::endl;
  }
  else
  {
    fbo_supported = false;
    std::cerr << "Video card does NOT support GL_ARB_framebuffer_object." << std::endl;
    return (false);
  }

  if(fbo_supported)
  {
    // create a framebuffer object, you need to delete them when program exits.
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // create a renderbuffer object to store color info
    glGenRenderbuffers(1, &renderbuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, width, height);
    // create a depthbuffer object to store depth info
    glGenRenderbuffers(1, &depthbuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depthbuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    // attach a texture to FBO color attachement point
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER, renderbuffer_);
    // attach a renderbuffer to depth attachment point
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthbuffer_);

    // check FBO status
    GLenum status;
    status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER);
    if(status != GL_FRAMEBUFFER_COMPLETE)
    {
      std::cerr << "Framebuffer error." << std::endl;
      return (false);
    }
  }
  return (true);
}

void CObjectModel::loadKeyframes(string obj_name)
{
  // Load keyframes: count the number of keyframes, then dynamically load jpeg images and pose parameters (xml)

  // release the previous data
  for(int i=0; i<num_keyframes_; i++)
  {
    cvReleaseImage(&keyframe_images_[i]);
    cvReleaseMat(&keyframe_poses_[i]);
    cvReleaseMat(&keyframe_keypoints_2d_[i]);
    cvReleaseMat(&keyframe_keypoints_3d_[i]);
    cvReleaseMat(&keyframe_descriptors_[i]);
  }

  num_keyframes_ = 0;
  std::string data_dir =  obj_name;
  fstream fsk;
  fsk.open((data_dir + "/" + "keyframe001.jpg").c_str());
  // count the number of keyframes
  while(fsk.is_open())
  {
    num_keyframes_++;
    fsk.close();
    std::stringstream ss;
    ss << data_dir << "/" << "keyframe" << std::setw(3) << std::setfill('0') << num_keyframes_ + 1 << ".jpg";
    fsk.open(ss.str().c_str());
  }

  //std::cout<<"(AKAN) Num of Keyframes: "<<num_keyframes_<<std::endl;

  // Read keyframes - jpg & xml files
  keyframe_images_.resize(num_keyframes_);
  keyframe_poses_.resize(num_keyframes_);
  keyframe_keypoints_2d_.resize(num_keyframes_);
  keyframe_keypoints_3d_.resize(num_keyframes_);
  keyframe_descriptors_.resize(num_keyframes_);

  std::stringstream ss;
  for(int i=0; i<num_keyframes_; i++)
  {
    ss.str(std::string()); // cleaning ss
    ss << data_dir << "/" << "keyframe" << std::setw(3) << std::setfill('0') << i + 1 << ".jpg";
    keyframe_images_[i] = cvLoadImage(ss.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    assert(keyframe_images_[i]);
    
    ss.str(std::string()); // cleaning ss
    ss << data_dir << "/" << "pose" << std::setw(3) << std::setfill('0') << i + 1 << ".xml";
    keyframe_poses_[i] = (CvMat*)cvLoad(ss.str().c_str());
    assert(keyframe_poses_[i]);
    
    ss.str(std::string()); // cleaning ss
    ss << data_dir << "/" << "keypoint2d" << std::setw(3) << std::setfill('0') << i + 1 << ".xml";
    keyframe_keypoints_2d_[i] = (CvMat*)cvLoad(ss.str().c_str());
    assert(keyframe_keypoints_2d_[i]);
    
    ss.str(std::string()); // cleaning ss
    ss << data_dir << "/" << "keypoint3d" << std::setw(3) << std::setfill('0') << i + 1 << ".xml";
    keyframe_keypoints_3d_[i] = (CvMat*)cvLoad(ss.str().c_str());
    assert(keyframe_keypoints_3d_[i]);
    
    ss.str(std::string()); // cleaning ss
    ss << data_dir << "/" << "descriptor" << std::setw(3) << std::setfill('0') << i + 1 << ".xml";
    keyframe_descriptors_[i] = (CvMat*)cvLoad(ss.str().c_str());
    assert(keyframe_descriptors_[i]);
  }
}

void CObjectModel::saveKeyframe(std::string obj_name, IplImage* imgG, CvMat* pose)
{
  // Save keyframe image, pose parameter, and SURF keypoints (2D keypoints, descriptors, 3D points)
  std::stringstream ss;

  // keyframe image
  std::string data_dir = obj_name;
  ss << data_dir << "/" << "keyframe" << std::setw(3) << std::setfill('0') << num_keyframes_+1 << ".jpg";
  cvSaveImage(ss.str().c_str(), imgG);
  
  // pose
  ss.str(std::string());
  ss << data_dir << "/" << "pose" << std::setw(3) << std::setfill('0') << num_keyframes_+1 << ".xml";
  cvSave(ss.str().c_str(), pose);

  vector<CvPoint2D32f> new2Dpt;
  vector<vector<float> > newDesc;
  vector<CvPoint3D32f> new3Dpt;    
  _saveSURFkeypoints(obj_name, imgG, pose, new2Dpt, newDesc, new3Dpt);
  assert(new2Dpt.size() == new3Dpt.size() && new3Dpt.size() == newDesc.size());

  if(new2Dpt.size()) // One or more keypoints
  {
    CvMat* pt2d = cvCreateMat(new2Dpt.size(), 1, CV_32FC2);
    CvMat* desc = cvCreateMat(newDesc.size(), newDesc[0].size(), CV_32F);
    //CvMat* pt3d = cvCreateMat(new3Dpt.size(), 1, CV_32FC3);
    CvMat* pt3d = cvCreateMat(new3Dpt.size(), 3, CV_32FC3);

    for(int i=0; i<new2Dpt.size(); i++)
      CV_MAT_ELEM(*pt2d, CvPoint2D32f, i, 0) = new2Dpt[i];

    for(int i=0; i<newDesc.size(); i++)
      for(int j=0; j<newDesc[i].size(); j++)
        CV_MAT_ELEM(*desc, float, i, j) = newDesc[i][j];

    for(int i=0; i<new3Dpt.size(); i++)
      CV_MAT_ELEM(*pt3d, CvPoint3D32f, i, 0) = new3Dpt[i];

    ss.str(std::string());
    ss << data_dir << "/" << "keypoint2d" << std::setw(3) << std::setfill('0') << num_keyframes_+1 << ".xml";
    cvSave(ss.str().c_str(), pt2d);

    ss.str(std::string());
    ss << data_dir << "/" << "keypoint3d" << std::setw(3) << std::setfill('0') << num_keyframes_+1 << ".xml";
    cvSave(ss.str().c_str(), pt3d);

    ss.str(std::string());
    ss << data_dir << "/" << "descriptor" << std::setw(3) << std::setfill('0') << num_keyframes_+1 << ".xml";
    cvSave(ss.str().c_str(), desc);

    keyframe_keypoints_2d_.push_back(pt2d);
    keyframe_descriptors_.push_back(desc);
    keyframe_keypoints_3d_.push_back(pt3d);
  }
  num_keyframes_++;

  // update keyframe databse
  IplImage* newKeyframe = cvCreateImage(cvSize(imgG->width, imgG->height), IPL_DEPTH_8U, 1);
  cvCopy(imgG, newKeyframe);
  keyframe_images_.push_back(newKeyframe);
  CvMat* newPose = cvCreateMat(pose->rows, pose->cols, CV_32F);
  cvCopy(pose, newPose);
  keyframe_poses_.push_back(newPose);
}

void CObjectModel::_saveSURFkeypoints(std::string& obj_name, IplImage* imgG, CvMat* pose, vector<CvPoint2D32f>& new2Dpt, vector<vector<float> >& newDesc, vector<CvPoint3D32f>& new3Dpt)
{
  // Facet-ID test
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  assert(meshmodel_->numtriangles < 16777216);
  for(int i=0; i<meshmodel_->numtriangles; i++)
  {
    unsigned char idx1 = (i+1) & 0x0000FF;
    unsigned char idx2 = ((i+1) & 0x00FF00) >> 8;
    unsigned char idx3 = ((i+1) & 0xFF0000) >> 16;

    glColor3ub(idx1, idx2, idx3);
    int v1 = meshmodel_->triangles[i].vindices[0];
    int v2 = meshmodel_->triangles[i].vindices[1];
    int v3 = meshmodel_->triangles[i].vindices[2];

    float x1 = meshmodel_->vertices[3*v1+0];
    float y1 = meshmodel_->vertices[3*v1+1];
    float z1 = meshmodel_->vertices[3*v1+2];

    float x2 = meshmodel_->vertices[3*v2+0];
    float y2 = meshmodel_->vertices[3*v2+1];
    float z2 = meshmodel_->vertices[3*v2+2];

    float x3 = meshmodel_->vertices[3*v3+0];
    float y3 = meshmodel_->vertices[3*v3+1];
    float z3 = meshmodel_->vertices[3*v3+2];

    //glBegin(GL_TRIANGLES);
    glBegin(GL_POLYGON);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glVertex3f(x3, y3, z3);
    glEnd();
  }

  // Extract SURF feature points
  CvSeq *imageKeypoints;
  CvSeq *imageDescriptors;
  CvMemStorage* ms = cvCreateMemStorage(0);
  CvSURFParams params = cvSURFParams(100, 3); // merged in the future
  cvExtractSURF(imgG, 0, &imageKeypoints, &imageDescriptors, ms, params);
  CvSeqReader reader, kreader;
  int dims = imageDescriptors->elem_size/sizeof(float);
  cvStartReadSeq( imageKeypoints, &kreader );
  cvStartReadSeq( imageDescriptors, &reader );
  vector<CvPoint2D32f> corner;
  vector<vector<float> > desc;
  // pre-allocate space
  corner.resize(imageDescriptors->total);
  desc.resize(imageDescriptors->total);
  for(int i=0; i<desc.size(); i++)
    desc[i].resize(dims);

  for(int i=0; i<imageDescriptors->total; i++ )
  {
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* descriptor = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    corner[i] = kp->pt;
    for(int j=0; j<dims; j++)
      desc[i][j] = descriptor[j];
  }

  CvMat* v = cvCreateMat(4, 1, CV_32F); // vector
  CvMat* nv = cvCreateMat(4, 1, CV_32F); // new vector
  CvMat* A = cvCreateMat(3, 3, CV_32F);
  CvMat* x = cvCreateMat(3, 1, CV_32F); // solution vector
  CvMat* bv = cvCreateMat(3, 1, CV_32F); // -1 vector
  float fx = CV_MAT_ELEM(*intrinsic_, float, 0, 0);
  float fy = CV_MAT_ELEM(*intrinsic_, float, 1, 1);
  float ux = CV_MAT_ELEM(*intrinsic_, float, 0, 2);
  float uy = CV_MAT_ELEM(*intrinsic_, float, 1, 2);
  CvMat* Minv = cvCreateMat(4, 4, CV_32F);

  IplImage* colorImage = cvCreateImage(cvSize(imgG->width, imgG->height), 8, 3);
  glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, colorImage->imageData);

  for(int i=0; i<corner.size(); i++)
  {
    unsigned char r = *(colorImage->imageData + 3*int(corner[i].x) + int(corner[i].y)*colorImage->widthStep + 0);
    unsigned char g = *(colorImage->imageData + 3*int(corner[i].x) + int(corner[i].y)*colorImage->widthStep + 1);
    unsigned char b = *(colorImage->imageData + 3*int(corner[i].x) + int(corner[i].y)*colorImage->widthStep + 2);
    int idx = r | (g << 8) | (b << 16);

    assert(0 <= idx && idx <= meshmodel_->numtriangles);

    // idx == 0 means black background.
    if(idx > 0 && idx <= meshmodel_->numtriangles) // any face
    //if(idx > 0) // any face
    {
      int v1 = meshmodel_->triangles[idx-1].vindices[0];
      int v2 = meshmodel_->triangles[idx-1].vindices[1];
      int v3 = meshmodel_->triangles[idx-1].vindices[2];

      float x1 = meshmodel_->vertices[3*v1+0];
      float y1 = meshmodel_->vertices[3*v1+1];
      float z1 = meshmodel_->vertices[3*v1+2];

      float x2 = meshmodel_->vertices[3*v2+0];
      float y2 = meshmodel_->vertices[3*v2+1];
      float z2 = meshmodel_->vertices[3*v2+2];

      float x3 = meshmodel_->vertices[3*v3+0];
      float y3 = meshmodel_->vertices[3*v3+1];
      float z3 = meshmodel_->vertices[3*v3+2];

      CV_MAT_ELEM(*v, float, 0, 0) = x1;
      CV_MAT_ELEM(*v, float, 1, 0) = y1;
      CV_MAT_ELEM(*v, float, 2, 0) = z1;
      CV_MAT_ELEM(*v, float, 3, 0) = 1;

      // Transform from object frame to camera frame (1st vertex)
      cvGEMM(pose, v, 1, NULL, 0, nv, 0);

      x1 = CV_MAT_ELEM(*nv, float, 0, 0);
      y1 = CV_MAT_ELEM(*nv, float, 1, 0);
      z1 = CV_MAT_ELEM(*nv, float, 2, 0);

      CV_MAT_ELEM(*v, float, 0, 0) = x2;
      CV_MAT_ELEM(*v, float, 1, 0) = y2;
      CV_MAT_ELEM(*v, float, 2, 0) = z2;
      CV_MAT_ELEM(*v, float, 3, 0) = 1;

      // Transform from object frame to camera frame (2nd vertex)
      cvGEMM(pose, v, 1, NULL, 0, nv, 0);

      x2 = CV_MAT_ELEM(*nv, float, 0, 0);
      y2 = CV_MAT_ELEM(*nv, float, 1, 0);
      z2 = CV_MAT_ELEM(*nv, float, 2, 0);

      CV_MAT_ELEM(*v, float, 0, 0) = x3;
      CV_MAT_ELEM(*v, float, 1, 0) = y3;
      CV_MAT_ELEM(*v, float, 2, 0) = z3;
      CV_MAT_ELEM(*v, float, 3, 0) = 1;

      // Transform from object frame to camera frame (3rd vertex)
      cvGEMM(pose, v, 1, NULL, 0, nv, 0);

      x3 = CV_MAT_ELEM(*nv, float, 0, 0);
      y3 = CV_MAT_ELEM(*nv, float, 1, 0);
      z3 = CV_MAT_ELEM(*nv, float, 2, 0);

      // Determine parameters of plane equation (a, b, c)
      CV_MAT_ELEM(*A, float, 0, 0) = x1;
      CV_MAT_ELEM(*A, float, 0, 1) = y1;
      CV_MAT_ELEM(*A, float, 0, 2) = z1;
      CV_MAT_ELEM(*A, float, 1, 0) = x2;
      CV_MAT_ELEM(*A, float, 1, 1) = y2;
      CV_MAT_ELEM(*A, float, 1, 2) = z2;
      CV_MAT_ELEM(*A, float, 2, 0) = x3;
      CV_MAT_ELEM(*A, float, 2, 1) = y3;
      CV_MAT_ELEM(*A, float, 2, 2) = z3;

      CV_MAT_ELEM(*bv, float, 0, 0) = -1;
      CV_MAT_ELEM(*bv, float, 1, 0) = -1;
      CV_MAT_ELEM(*bv, float, 2, 0) = -1;

      cvInvert(A, A, CV_SVD); // get inv(A)

      cvGEMM(A, bv, 1, NULL, 0, x, 0); // x = inv(A)*b

      float a = CV_MAT_ELEM(*x, float, 0, 0);
      float b = CV_MAT_ELEM(*x, float, 1, 0);
      float c = CV_MAT_ELEM(*x, float, 2, 0);

      // Determine 3D coordinates of 3D corner point in camera frame
      CvPoint3D32f pt3d;
      pt3d.z = -1/(a*float(corner[i].x-ux)/fx + b*float(corner[i].y-uy)/fy + c);
      pt3d.x = float(corner[i].x-ux)*pt3d.z/fx;
      pt3d.y = float(corner[i].y-uy)*pt3d.z/fy;

      // Transform from camera frame to object frame
      CV_MAT_ELEM(*A, float, 0, 0) = CV_MAT_ELEM(*pose, float, 0, 0);
      CV_MAT_ELEM(*A, float, 0, 1) = CV_MAT_ELEM(*pose, float, 1, 0);
      CV_MAT_ELEM(*A, float, 0, 2) = CV_MAT_ELEM(*pose, float, 2, 0);
      CV_MAT_ELEM(*A, float, 1, 0) = CV_MAT_ELEM(*pose, float, 0, 1);
      CV_MAT_ELEM(*A, float, 1, 1) = CV_MAT_ELEM(*pose, float, 1, 1);
      CV_MAT_ELEM(*A, float, 1, 2) = CV_MAT_ELEM(*pose, float, 2, 1);
      CV_MAT_ELEM(*A, float, 2, 0) = CV_MAT_ELEM(*pose, float, 0, 2);
      CV_MAT_ELEM(*A, float, 2, 1) = CV_MAT_ELEM(*pose, float, 1, 2);
      CV_MAT_ELEM(*A, float, 2, 2) = CV_MAT_ELEM(*pose, float, 2, 2);
      // Translate
      pt3d.x -= CV_MAT_ELEM(*pose, float, 0, 3);
      pt3d.y -= CV_MAT_ELEM(*pose, float, 1, 3);
      pt3d.z -= CV_MAT_ELEM(*pose, float, 2, 3);
      // Rotate
      CV_MAT_ELEM(*bv, float, 0, 0) = pt3d.x;
      CV_MAT_ELEM(*bv, float, 1, 0) = pt3d.y;
      CV_MAT_ELEM(*bv, float, 2, 0) = pt3d.z;

      cvGEMM(A, bv, 1, NULL, 0, x, 0); // x = A'*b (inverse rotation)

      pt3d.x = CV_MAT_ELEM(*x, float, 0, 0);
      pt3d.y = CV_MAT_ELEM(*x, float, 1, 0);
      pt3d.z = CV_MAT_ELEM(*x, float, 2, 0);

      new2Dpt.push_back(corner[i]);
      new3Dpt.push_back(pt3d);
      newDesc.push_back(desc[i]);
    }
  }
  cvReleaseImage(&colorImage);
  cvReleaseMat(&v);
  cvReleaseMat(&nv);
  cvReleaseMat(&A);
  cvReleaseMat(&x);
  cvReleaseMat(&bv);
  cvReleaseMat(&Minv);

  glPopMatrix();
  //glutSwapBuffers();
  glFlush();
}

void CObjectModel::loadEdgeTemplates(string obj_name)
{
  // Release the previous data
  for(int i=0; i<num_edge_templates_; i++)
  {
    cvReleaseMat(&edge_template_poses_[i]);
  }

  num_edge_templates_ = 0;

  string templateFileName(obj_name + "/" + obj_name + ".txt");

  fstream file;
  file.open(templateFileName.c_str());

  // read the input template list
  if(!file.is_open())
  {
    cout << "No edge template data in " << templateFileName << std::endl;
  }
  else
  {
    string line;
    getline(file, line);
    num_edge_templates_ = atoi(line.c_str());

    std::cout << "num_edge_templates_ " << num_edge_templates_ << std::endl;



    // Read templates - png & xml files
    string data_dir =  obj_name;
    char buf[50];
    edge_template_poses_.resize(num_edge_templates_);
    for(int i=0; i<num_edge_templates_; i++)
    {
      sprintf(buf, "edge_template_pose%03d.xml", i);
      edge_template_poses_[i] = (CvMat*)cvLoad((data_dir + "/" + buf).c_str());
      assert(edge_template_poses_[i]);
    }
  }
}

void CObjectModel::findVisibleSamplePoints(void)
{
  glPushMatrix();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Draw the face (fill) with offset
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0, 1.0);
  glColor3f(0.0, 0.0, 0.0);
  glCallList(dl_); // draw object model saved in display list

  glDisable(GL_POLYGON_OFFSET_FILL);
  // Occlusion test
  vector<CvPoint3D32f> visible_sharpedge_samplepoint; // sample points after visibility test
  vector<int> visible_sharpedge_samplepoint_edge_membership;
  vector<GLuint> vQueries;
  int i;
  int N = 0; // number of test points
  if(dulledge_)
  {
    //#pragma omp parallel for private(N)
    for(i=0; i<set_dull_sample_points_.size(); i++)
    {
      N += (int)set_dull_sample_points_[i].size();
    }
  }
  N += (int)sharp_sample_points_.size();

  Timer timer;
  timer.start();

  assert(N > 0);
  vQueries.resize(N);
  int total_passed = 0;
  glGenQueriesARB(N, &vQueries[0]);
  // Turn on occlusion testing
  glColorMask(0, 0, 0, 0);
  glDepthMask(GL_FALSE);
  glPointSize(1.0);

  int k = 0;
  if(dulledge_)
  {
    for(i=0; i<set_dull_sample_points_.size(); i++)
    {
      for(int j=0; j<set_dull_sample_points_[i].size(); j++)
      {
        glBeginQueryARB(GL_SAMPLES_PASSED_ARB, vQueries[k++]);
        glBegin(GL_POINTS);
        glVertex3f(set_dull_sample_points_[i][j].x, set_dull_sample_points_[i][j].y, set_dull_sample_points_[i][j].z);
        glEnd();
        glEndQueryARB(GL_SAMPLES_PASSED_ARB);
      }
    }
  }
  
  for(i=0; i<sharp_sample_points_.size(); i++)
  {
    glBeginQueryARB(GL_SAMPLES_PASSED_ARB, vQueries[k++]);
    glBegin(GL_POINTS);
    glVertex3f(sharp_sample_points_[i].x, sharp_sample_points_[i].y, sharp_sample_points_[i].z);
    glEnd();
    glEndQueryARB(GL_SAMPLES_PASSED_ARB);
  }

  i = N*3/4; // instead of N-1, to prevent the GPU from going idle
  GLint ready;
  do {
    // do useful work here, if any
    // ..
    glGetQueryObjectivARB(vQueries[i], GL_QUERY_RESULT_AVAILABLE_ARB, &ready);
  } while (!ready);

  // turn off occlusion testing
  glColorMask(1, 1, 1, 1);
  glDepthMask(GL_TRUE);

  k = 0; // start index
  vector<CvPoint3D32f> visible_dulledge_samplepoint;
  vector<int> visible_boundarydulledge_samplepoint_edge_membership;
  GLuint ps;
  vector<vector<int> > visible_dulledge_samplepoint_flags;
  visible_dulledge_samplepoint_flags.resize(set_dull_sample_points_.size());

  if(dulledge_)
  {
    for(i=0; i<set_dull_sample_points_.size(); i++)
    {
      visible_dulledge_samplepoint_flags[i].resize(set_dull_sample_points_[i].size());
      // visible edge!
      for(int j=0; j<set_dull_sample_points_[i].size(); j++)
      {
        glGetQueryObjectuivARB(vQueries[k++], GL_QUERY_RESULT_ARB, &ps);
        assert(ps == 0 || ps == 1);
        visible_dulledge_samplepoint_flags[i][j] = (int)ps;
      }
    }
  }

  visible_sharpedge_samplepoint.clear();
  visible_sharpedge_samplepoint_edge_membership.clear();
  GLuint passed;
  for(i=0; i<sharp_sample_points_.size(); i++)
  {
    glGetQueryObjectuivARB(vQueries[k++], GL_QUERY_RESULT_ARB, &passed);
    assert(passed == 0 || passed == 1);
    if(passed == 1)
    {
      visible_sharpedge_samplepoint.push_back(sharp_sample_points_[i]);
      visible_sharpedge_samplepoint_edge_membership.push_back(sharp_sample_points_edge_indices_[i]);
    }
    total_passed += passed;
  }

  glDeleteQueriesARB(N, &vQueries[0]);

  //timer.printTimeMilliSec("occlusion query");

  if(dulledge_)
  {
    // rotate face normal using current rotation of pose info.
    vector<CvPoint3D32f> uepn1, uepn2, dep_tranf;
    CvMat* vertices = cvCreateMat(4,4, CV_32F);
    CvMat* vertices_transf = cvCreateMat(4,4, CV_32F);
    // rotate all face normal with respect to the current pose (rotation only)
    uepn1.resize(dull_normals_.size());
    uepn2.resize(dull_normals_.size());
    dep_tranf.resize(dull_normals_.size());
    
    for(i = 0; i < dull_normals_.size(); i++)
    {
      CV_MAT_ELEM(*vertices, float, 0, 0) = dull_edges_[i].first.x;
      CV_MAT_ELEM(*vertices, float, 1, 0) = dull_edges_[i].first.y;
      CV_MAT_ELEM(*vertices, float, 2, 0) = dull_edges_[i].first.z;
      CV_MAT_ELEM(*vertices, float, 3, 0) = 1.0f;

      CV_MAT_ELEM(*vertices, float, 0, 1) = dull_edges_[i].second.x;
      CV_MAT_ELEM(*vertices, float, 1, 1) = dull_edges_[i].second.y;
      CV_MAT_ELEM(*vertices, float, 2, 1) = dull_edges_[i].second.z;
      CV_MAT_ELEM(*vertices, float, 3, 1) = 1.0f;

      CV_MAT_ELEM(*vertices, float, 0, 2) = dull_normals_[i].first.x;
      CV_MAT_ELEM(*vertices, float, 1, 2) = dull_normals_[i].first.y;
      CV_MAT_ELEM(*vertices, float, 2, 2) = dull_normals_[i].first.z;
      CV_MAT_ELEM(*vertices, float, 3, 2) = 1.0f;

      CV_MAT_ELEM(*vertices, float, 0, 3) = dull_normals_[i].second.x;
      CV_MAT_ELEM(*vertices, float, 1, 3) = dull_normals_[i].second.y;
      CV_MAT_ELEM(*vertices, float, 2, 3) = dull_normals_[i].second.z;
      CV_MAT_ELEM(*vertices, float, 3, 3) = 1.0f;

      cvMatMul(pose_, vertices, vertices_transf);
      CvPoint3D32f v10, v11, v12, v20, v21, v22;
      v10.x = CV_MAT_ELEM(*vertices_transf, float, 0, 0);
      v10.y = CV_MAT_ELEM(*vertices_transf, float, 1, 0);
      v10.z = CV_MAT_ELEM(*vertices_transf, float, 2, 0);

      v11.x = CV_MAT_ELEM(*vertices_transf, float, 0, 1);
      v11.y = CV_MAT_ELEM(*vertices_transf, float, 1, 1);
      v11.z = CV_MAT_ELEM(*vertices_transf, float, 2, 1);

      v12.x = CV_MAT_ELEM(*vertices_transf, float, 0, 2);
      v12.y = CV_MAT_ELEM(*vertices_transf, float, 1, 2);
      v12.z = CV_MAT_ELEM(*vertices_transf, float, 2, 2);

      v20 = v11;
      v21 = v10;
      v22.x = CV_MAT_ELEM(*vertices_transf, float, 0, 3);
      v22.y = CV_MAT_ELEM(*vertices_transf, float, 1, 3);
      v22.z = CV_MAT_ELEM(*vertices_transf, float, 2, 3);

      // calculate face normals
      float u[3];
      float v[3];
      float n[3];

      u[0] = v11.x - v10.x;
      u[1] = v11.y - v10.y;
      u[2] = v11.z - v10.z;
      v[0] = v12.x - v10.x;
      v[1] = v12.y - v10.y;
      v[2] = v12.z - v10.z;
      Cross(u, v, n); // vector cross product for calculating face normal vector
      uepn1[i].x = n[0];
      uepn1[i].y = n[1];
      uepn1[i].z = n[2];

      u[0] = v21.x - v20.x;
      u[1] = v21.y - v20.y;
      u[2] = v21.z - v20.z;
      v[0] = v22.x - v20.x;
      v[1] = v22.y - v20.y;
      v[2] = v22.z - v20.z;
      Cross(u, v, n);
      uepn2[i].x = n[0];
      uepn2[i].y = n[1];
      uepn2[i].z = n[2];

      dep_tranf[i] = v10; // save for calculating viewing vector (v10 is one of the shared vertex)
    }
    cvReleaseMat(&vertices);
    cvReleaseMat(&vertices_transf);

    // determine visibility of dull edges : test face normal of two triangles that share the dull edge
    dull_boundary_edges_.clear();

    assert(set_dull_sample_points_.size() == dull_edges_.size());

    dull_boundary_sample_points_edge_indices_.clear();

    for(i = 0; i < set_dull_sample_points_.size(); i++)
    {
      // determine visibility of 1st face
      CvPoint3D32f vv; // viewing vector (a vector from camera position to any point on the surface)
      vv = dep_tranf[i];
      // calculate inner product (triangle 1)
      float ip1 = uepn1[i].x*vv.x + uepn1[i].y*vv.y + uepn1[i].z*vv.z;
      // determine visibility of 2nd face 
      // calculate inner product (triangle 1)
      float ip2 = uepn2[i].x*vv.x + uepn2[i].y*vv.y + uepn2[i].z*vv.z;

      if(ip1*ip2 <= 0)    // invisible edge would be visible
      {
        // this dull edge is visible!
        // save the sample points along this visible dull edge
        int bdep_idx = dull_boundary_edges_.size() + sharp_edges_.size();
        
        for(int j=0; j<set_dull_sample_points_[i].size(); j++)
        {
          if(visible_dulledge_samplepoint_flags[i][j])
          {
            visible_dulledge_samplepoint.push_back(set_dull_sample_points_[i][j]);
            dull_boundary_sample_points_edge_indices_.push_back(bdep_idx);
          }
        }
        // save boundary dull edge points (start, end)
        dull_boundary_edges_.push_back(dull_edges_[i]);
      }
    }
    uepn1.clear();
    uepn2.clear();
  }

  visible_sample_points_.resize(visible_sharpedge_samplepoint.size()+visible_dulledge_samplepoint.size());
  for(i=0; i<int(visible_sharpedge_samplepoint.size()); i++)
  {
    visible_sample_points_[i].coord3 = visible_sharpedge_samplepoint[i];
    visible_sample_points_[i].coord2 = project3Dto2D(visible_sample_points_[i].coord3);
    visible_sample_points_[i].edge_mem = visible_sharpedge_samplepoint_edge_membership[i];
  }
  k = i; // start_index
  for(i=0; i<int(visible_dulledge_samplepoint.size()); i++)
  {
    visible_sample_points_[k+i].coord3 = visible_dulledge_samplepoint[i];
    visible_sample_points_[k+i].coord2 = project3Dto2D(visible_sample_points_[k+i].coord3);
    visible_sample_points_[k+i].edge_mem = dull_boundary_sample_points_edge_indices_[i];
  }

  glPopMatrix();
  //glutSwapBuffers();
  //glFlush();

	// remove all points that are out of
	// visible z-area
  /*std::vector<SamplePoint> newSamplepoints;
  for(int i=0; i< visible_sample_points_.size(); i++)
	{
		//std::cout << visible_sample_points_[i].coord2.x ;
		unsigned short val = depth.at<unsigned short>(visible_sample_points_[i].coord2.y,visible_sample_points_[i].coord2.x);

		if(val < max && val > min)
				newSamplepoints.push_back(visible_sample_points_[i]);
	}
	visible_sample_points_ = newSamplepoints;*/

  visible_sharpedge_samplepoint.clear();
  visible_sharpedge_samplepoint_edge_membership.clear();
  visible_dulledge_samplepoint.clear();
}

CvPoint2D32f CObjectModel::project3Dto2D(CvPoint3D32f &pt3)
{
  return project3Dto2D(pt3, pose_);
}

CvPoint2D32f CObjectModel::project3Dto2D(CvPoint3D32f &pt3, CvMat* pose)
{
  CvPoint2D32f pt2;
  CvPoint3D32f pt3_cam;
  pt3_cam.x = CV_MAT_ELEM(*pose, float, 0, 0)*pt3.x + CV_MAT_ELEM(*pose, float, 0, 1)*pt3.y + CV_MAT_ELEM(*pose, float, 0, 2)*pt3.z + CV_MAT_ELEM(*pose, float, 0, 3);
  pt3_cam.y = CV_MAT_ELEM(*pose, float, 1, 0)*pt3.x + CV_MAT_ELEM(*pose, float, 1, 1)*pt3.y + CV_MAT_ELEM(*pose, float, 1, 2)*pt3.z + CV_MAT_ELEM(*pose, float, 1, 3);
  pt3_cam.z = CV_MAT_ELEM(*pose, float, 2, 0)*pt3.x + CV_MAT_ELEM(*pose, float, 2, 1)*pt3.y + CV_MAT_ELEM(*pose, float, 2, 2)*pt3.z + CV_MAT_ELEM(*pose, float, 2, 3);

  float fx = CV_MAT_ELEM(*intrinsic_, float, 0, 0);
  float fy = CV_MAT_ELEM(*intrinsic_, float, 1, 1);
  float ux = CV_MAT_ELEM(*intrinsic_, float, 0, 2);
  float uy = CV_MAT_ELEM(*intrinsic_, float, 1, 2);

  pt2.x = fx*pt3_cam.x/pt3_cam.z + ux;
  pt2.y = fy*pt3_cam.y/pt3_cam.z + uy;

  if(pt3_cam.z == 0.0f)
  {
    pt2.x = pt2.y = numeric_limits<float>::infinity();
  }
  else
  {
    float th_max = 10000.0;
    float th_min = -10000.0;
    pt2.x = fx*pt3_cam.x/pt3_cam.z + ux;
    pt2.y = fy*pt3_cam.y/pt3_cam.z + uy;
    if(pt2.x < th_min || pt2.x > th_max)
      pt2.x = numeric_limits<float>::infinity();
    if(pt2.y < th_min || pt2.y > th_max)
      pt2.y = numeric_limits<float>::infinity();
  }

  return pt2;
}

void CObjectModel::setProjectionMatrix(CvMat* intrinsic)
{
  glMatrixMode(GL_PROJECTION);
  // intrinsic parameters are updated (should be linked with camera class)
  float fx = CV_MAT_ELEM(*intrinsic, float, 0, 0);
  float fy = CV_MAT_ELEM(*intrinsic, float, 1, 1);
  float ux = CV_MAT_ELEM(*intrinsic, float, 0, 2);
  float uy = CV_MAT_ELEM(*intrinsic, float, 1, 2);

  float near_ = 0.1f;   // minimum depth: 10 cm
  float far_ = 30.f;     // maximum depth: 50 m

  GLfloat Mp[16] = {
    2.f*fx/(float)width_, 0.f, 0.f, 0.f,
    0.f, -2.f*fy/(float)height_, 0.f, 0.f,
    2.0f*ux/(float)width_ - 1.f, -2.f*uy/(float)height_ + 1.f, (far_+near_)/(far_-near_), 1.f,
    0.f, 0.f, -2.f*far_*near_/(far_-near_), 0.f
  };

  glLoadMatrixf(Mp);

  cvCopy(intrinsic, intrinsic_);
}

void CObjectModel::setModelviewMatrix(CvMat* pose)
{
//  printf("reached 0\n"); fflush(stdout);
  GLenum e = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
  if (e != GL_FRAMEBUFFER_COMPLETE)
    printf("There is a problem with the FBO\n");
  glMatrixMode(GL_MODELVIEW);
  GLfloat Mm[16];

  assert(pose);
  cvCopy(pose, pose_);
  CvMat* poset = cvCreateMat(4, 4, CV_32F);
  // model view matrix is row-cols exchanged
  cvTranspose(pose, poset);
  memcpy(Mm, poset->data.fl, sizeof(float)*16);
  cvReleaseMat(&poset);

  glLoadMatrixf(Mm);
}

void CObjectModel::findNormalUsingEdgeCoord(void)
{
  if(use_fine_orientation_)
    findNormalUsingEdgeCoordFineOri();
  else
    findNormalUsingEdgeCoordCoarseOri();
}

void CObjectModel::findNormalUsingEdgeCoordCoarseOri(void)
{
  // calculate edge orientation from 3D coord
  CvPoint2D32f p1;
  CvPoint2D32f p2;
  vector<unsigned char> nidx;
  vector<float> deg;
  deg.resize(sharp_edges_.size() + dull_boundary_edges_.size());
  nidx.resize(sharp_edges_.size() + dull_boundary_edges_.size());

//#pragma omp parallel for
  for(int i=0; i<(int)sharp_edges_.size(); i++)
  {
    p1 = project3Dto2D(sharp_edges_[i].first);
    p2 = project3Dto2D(sharp_edges_[i].second);
    CvPoint2D32f dp;
    dp.x = p2.x - p1.x;
    dp.y = p2.y - p1.y;
    deg[i] = atan2(dp.y, dp.x) + M_PI/2.0;
    deg[i] *= static_cast<float>(180.0f/M_PI); // deg[i] is in -90 ~ 270
    //assert(-90.0 <= deg[i] && deg[i] <= 270.0);

    // Normal direction
    // 0: |
    // 1: /
    // 2: -
    // 3: '\'
    nidx[i] = 4;
    if(deg[i] < -67.5)                          nidx[i] = 0;
    else if(-67.5 <= deg[i] && deg[i] < -22.5)  nidx[i] = 1;
    else if(-22.5 <= deg[i] && deg[i] <  22.5)  nidx[i] = 2;
    else if( 22.5 <= deg[i] && deg[i] <  67.5)  nidx[i] = 3;
    else if( 67.5 <= deg[i] && deg[i] < 112.5)  nidx[i] = 0;
    else if(112.5 <= deg[i] && deg[i] < 157.5)  nidx[i] = 1;
    else if(157.5 <= deg[i] && deg[i] < 202.5)  nidx[i] = 2;
    else if(202.5 <= deg[i] && deg[i] < 247.5)  nidx[i] = 3;
    else if(247.5 <= deg[i])                    nidx[i] = 0;
  }

  int j = sharp_edges_.size();

//#pragma omp parallel for
  for(int i=0; i<(int)dull_boundary_edges_.size(); i++)
  {
    p1 = project3Dto2D(dull_boundary_edges_[i].first);
    p2 = project3Dto2D(dull_boundary_edges_[i].second);
    CvPoint2D32f dp;
    dp.x = p2.x - p1.x;
    dp.y = p2.y - p1.y;
    deg[i+j] = atan2(dp.y, dp.x) + M_PI/2.0;
    deg[i+j] *= static_cast<float>(180.0f/M_PI); // deg[i] is in -90 ~ 270
    //assert(-90.0 <= deg[i+j] && deg[i+j] <= 270.0);

    // Normal direction
    // 0: |
    // 1: /
    // 2: -
    // 3: '\'
    nidx[i+j] = 4;
    if(deg[i+j] < -67.5)                            nidx[i+j] = 0;
    else if(-67.5 <= deg[i+j] && deg[i+j] < -22.5)  nidx[i+j] = 1;
    else if(-22.5 <= deg[i+j] && deg[i+j] <  22.5)  nidx[i+j] = 2;
    else if( 22.5 <= deg[i+j] && deg[i+j] <  67.5)  nidx[i+j] = 3;
    else if( 67.5 <= deg[i+j] && deg[i+j] < 112.5)  nidx[i+j] = 0;
    else if(112.5 <= deg[i+j] && deg[i+j] < 157.5)  nidx[i+j] = 1;
    else if(157.5 <= deg[i+j] && deg[i+j] < 202.5)  nidx[i+j] = 2;
    else if(202.5 <= deg[i+j] && deg[i+j] < 247.5)  nidx[i+j] = 3;
    else if(247.5 <= deg[i+j])                      nidx[i+j] = 0;
  }

//#pragma omp parallel for
  for(int i=0; i<int(visible_sample_points_.size()); i++)
  {
    int edge_mem = visible_sample_points_[i].edge_mem;
    unsigned char _nidx = nidx[edge_mem];

    assert(_nidx >= 0 && _nidx <= 3);
    visible_sample_points_[i].nidx = _nidx;
    if(_nidx == 0)        visible_sample_points_[i].normal_ang = M_PI/2;
    else if(_nidx == 1)   visible_sample_points_[i].normal_ang = -M_PI/4;
    else if(_nidx == 2)   visible_sample_points_[i].normal_ang = 0;
    else if(_nidx == 3)   visible_sample_points_[i].normal_ang = M_PI/4;

    visible_sample_points_[i].normal_ang_deg = deg[edge_mem];
  }
}

void CObjectModel::findNormalUsingEdgeCoordFineOri(void)
{
  // calculate edge orientation from 3D coord
  vector<float> deg;
  vector<double> dx;
  vector<double> dy;

  deg.resize(sharp_edges_.size() + dull_boundary_edges_.size());
  dx.resize(sharp_edges_.size() + dull_boundary_edges_.size());
  dy.resize(sharp_edges_.size() + dull_boundary_edges_.size());

  int sep_size = sharp_edges_.size();
//#pragma omp parallel for
  for(int i=0; i<sep_size; i++)
  {
    CvPoint2D32f p1 = project3Dto2D(sharp_edges_[i].first);
    CvPoint2D32f p2 = project3Dto2D(sharp_edges_[i].second);
    CvPoint2D32f dp;
    dp.x = p2.x - p1.x;
    dp.y = p2.y - p1.y;
    double ori_rad = atan2(dp.y, dp.x) + M_PI/2.0;
    dx[i] = cos(ori_rad);
    dy[i] = sin(ori_rad);
    deg[i] = ori_rad*180.0f/M_PI; // deg[i] is in -90 ~ 270
  }

  int bdep_size = dull_boundary_edges_.size();
//#pragma omp parallel for
  for(int i=0; i<bdep_size; i++)
  {
    CvPoint2D32f p1 = project3Dto2D(dull_boundary_edges_[i].first);
    CvPoint2D32f p2 = project3Dto2D(dull_boundary_edges_[i].second);
    CvPoint2D32f dp;
    dp.x = p2.x - p1.x;
    dp.y = p2.y - p1.y;
    double ori_rad = atan2(dp.y, dp.x) + M_PI/2.0;
    dx[i+sep_size] = cos(ori_rad);
    dy[i+sep_size] = sin(ori_rad);
    deg[i+sep_size] = ori_rad*180.0f/M_PI; // deg[i] is in -90 ~ 270
  }

//#pragma omp parallel for
  for(int i=0; i<int(visible_sample_points_.size()); i++)
  {
    int edge_mem = visible_sample_points_[i].edge_mem;

    visible_sample_points_[i].normal_ang_deg = deg[edge_mem];
    visible_sample_points_[i].dx = dx[edge_mem];
    visible_sample_points_[i].dy = dy[edge_mem];
  }
}

void CObjectModel::extractEdgeOri(IplImage* img, int smoothSize/*=1*/)
{
  IplImage* imgG = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_32F, 1);

  if(img->nChannels == 1) // Gray image
    cvConvert(img, imgG);
  else
    cvCvtColor(img, imgG, CV_BGR2GRAY);

  // cvSmooth taks about 4 ms, skip it if it is possible
  if(smoothSize > 0)
    cvSmooth(imgG, imgG, CV_GAUSSIAN, smoothSize, smoothSize);

  cvSobel(imgG, img_gx_, 1, 0, 3); // 1st order derivative in x direction
  cvSobel(imgG, img_gy_, 0, 1, 3); // 1st order derivative in y direction
  cvReleaseImage(&imgG);
}

void CObjectModel::extractEdge(IplImage* img, int smoothSize/*=1*/, int cannyLow/*=20*/, int cannyHigh/*=40*/, IplImage* edge/*=NULL*/)
{
  assert(img->nChannels == 1);

  // Use Canny edge detector and calculate error (d)
  IplImage* imgG = cvCreateImage(cvSize(img->width, img->height), 8, 1);

  if(img->nChannels == 1) // Gray image
    cvCopy(img, imgG);
  else
  {
    cvCvtColor(img, imgG, CV_BGR2GRAY);
  }

  if(edge == NULL)
  {
    // Smooth image first

    if(smoothSize > 0)
      cvSmooth(imgG, imgG, CV_GAUSSIAN, smoothSize, smoothSize);

    // Extract Canny edges
    // GM shield model: 20, 120
    // ICRA exp setting: 20, 40
    // ICCV'11 transparent object: 20, 60
    cvCanny(imgG, img_edge_, cannyLow, cannyHigh);
  }
  else
  {
    // Copy edges and use it
    cvCopy(edge, img_edge_);
  }

  cvReleaseImage(&imgG);
}

bool CObjectModel::_withinOri(float o1, float o2, float oth)
{
  // both o1 and o2 are in degree
  float diff_o = o1 - o2;
  while(diff_o < -90.f)
    diff_o += 180.f;

  while(diff_o > 90.f)
    diff_o -= 180.f;

  if(diff_o > -oth && diff_o < oth)
    return true;
  else
    return false;
}

void CObjectModel::drawPointsAndErrorCoarseOri(IplImage* img_dest)
{
  cvCvtColor(img_edge_, img_dest, CV_GRAY2BGR); // copy edge data

  for(int i=0; i<int(visible_sample_points_.size()); i++)
  {
    if(visible_sample_points_[i].dist < maxd_) // only valid points
    {
      cvLine(img_dest, 
        cvPoint(int(visible_sample_points_[i].coord2.x), int(visible_sample_points_[i].coord2.y)), 
        cvPoint(
        int(visible_sample_points_[i].coord2.x + visible_sample_points_[i].dist*visible_sample_points_[i].nuv.x), 
        int(visible_sample_points_[i].coord2.y + visible_sample_points_[i].dist*visible_sample_points_[i].nuv.y)
        ), CV_RGB(255, 0, 0), 1, draw_type_, 0);
      cvCircle(img_dest, cvPointFrom32f(visible_sample_points_[i].coord2), 1, CV_RGB(0,255,0), -1, draw_type_, 0);
    }
  }
}

void CObjectModel::drawPointsAndErrorFineOri(IplImage* img_dest)
{
  cvCvtColor(img_edge_, img_dest, CV_GRAY2BGR); // copy edge data

  for(int i=0; i<int(visible_sample_points_.size()); i++)
  {
    if(visible_sample_points_[i].dist < maxd_) // only valid points
    {
      //if(visible_sample_points_[i].edge_pt2.x <= 0.f)
      //  std::cout << "a" << std::endl;

      if(visible_sample_points_[i].edge_pt2.x == 0.f && visible_sample_points_[i].edge_pt2.y == 0.f)
        continue;

      cvLine(img_dest, 
        cvPoint(int(visible_sample_points_[i].coord2.x), int(visible_sample_points_[i].coord2.y)), 
        cvPoint(int(visible_sample_points_[i].edge_pt2.x), int(visible_sample_points_[i].edge_pt2.y)), 
        CV_RGB(255, 0, 0), 1, draw_type_, 0
        );
      cvCircle(img_dest, cvPointFrom32f(visible_sample_points_[i].coord2), 1, CV_RGB(0,255,0), -1, draw_type_, 0);
    }
  }
}

void CObjectModel::findEdgeCorrespondences()
{
  if(use_fine_orientation_)
    findEdgeCorrespondencesFineOri();
  else
    findEdgeCorrespondencesCoarseOri();
}

void CObjectModel::findEdgeCorrespondencesCoarseOri()
{
  const float oth = 15.f; // +- 15 degree
  const double max_dist = static_cast<double>(maxd_);

//#pragma omp parallel for
  for(int i = 0; i < static_cast<int>(visible_sample_points_.size()); i++)
  {
    int max_step;
    int max_diag_step = int(double(maxd_/2)*sqrt(2.));

    char dx = 0;
    char dy = 0;
    unsigned char norm_idx = visible_sample_points_[i].nidx; // Get normal index of i-th point
    switch(norm_idx)
    {
    case 0: // |
      dx = 0;
      dy = 1;
      max_step = maxd_;
      break;
    case 1: // /
      dx = 1;
      dy = -1;
      max_step = max_diag_step;
      break;
    case 2: // -
      dx = 1;
      dy = 0;
      max_step = maxd_;
      break;
    case 3: // '\'
      dx = 1;
      dy = 1;
      max_step = max_diag_step;
      break;
    default:
      printf("error in normal index");
      exit(-1);
    }

    int x = int(visible_sample_points_[i].coord2.x);
    int y = int(visible_sample_points_[i].coord2.y);

    // avoid invisible 2d points
    if(x < 0 || x >= width_)
      continue;
    if(y < 0 || y >= height_)
      continue;
    
    // check both direction
    double min_dist1 = max_dist;
    double min_dist2 = max_dist;
    bool valid_indices;

    // direction 1
    valid_indices = true;
    for(int j=0; j<max_step && valid_indices; j++)
    {
      if(*(img_edge_->imageData + (x) + (y)*img_edge_->widthStep) == char(255)) // find edge
      {
        const float* ptr_gx = (const float*)(img_gx_->imageData + y*img_gx_->widthStep);
        const float* ptr_gy = (const float*)(img_gy_->imageData + y*img_gy_->widthStep);
        float ori = atan2(ptr_gy[x], ptr_gx[x])*180.f/M_PI;
        if(_withinOri(ori, visible_sample_points_[i].normal_ang_deg, oth))
        {
          min_dist1 = j;
          break;
        }
      }
      x += dx;
      y += dy;
      // Determine the new indices are valid or not
      if(x < 0 || x >= width_ || y < 0 || y >= height_)
        valid_indices = false;
    }

    // direction 2 (opposite direction)
    dx = dx * -1;
    dy = dy * -1;
    x = int(visible_sample_points_[i].coord2.x);
    y = int(visible_sample_points_[i].coord2.y);
    valid_indices = true;
    for(int j=1; j<max_step && valid_indices; j++)
    {
      if(*(img_edge_->imageData + (x) + (y)*img_edge_->widthStep) == char(255)) // find edge
      {
        const float* ptr_gx = (const float*)(img_gx_->imageData + y*img_gx_->widthStep);
        const float* ptr_gy = (const float*)(img_gy_->imageData + y*img_gy_->widthStep);
        float ori = atan2(ptr_gy[x], ptr_gx[x])*180.f/M_PI;
        if(_withinOri(ori, visible_sample_points_[i].normal_ang_deg, oth))
        {
          min_dist2 = j;
          break;
        }
      }
      x += dx;
      y += dy;
      // Determine the new indices are valid or not
      if(x < 0 || x >= width_ || y < 0 || y >= height_)
        valid_indices = false;
    }

    double min_dist = (min_dist1 < min_dist2) ? min_dist1 : min_dist2;

    if(min_dist < max_dist)
    {
      // also update the normal direction more precisely according to the machted edge direction
      if(min_dist == min_dist1)
        ; // Stay same
      else
        visible_sample_points_[i].normal_ang += M_PI; // Opposite direction, plus M_PI

      // if the distance is valid and \ or /, then convert to real distance (not the step)
      if(norm_idx == 1 || norm_idx == 3)
        min_dist = sqrt(2.)*min_dist;
    }

    visible_sample_points_[i].nuv = cvPoint2D32f(cos(visible_sample_points_[i].normal_ang), sin(visible_sample_points_[i].normal_ang));
    visible_sample_points_[i].dist = min_dist;
  }
}

void CObjectModel::findEdgeCorrespondencesFineOri()
{
  const float oth = 15.f; // +- 15 degree
  const int max_step = static_cast<int>(maxd_);

//#pragma omp parallel for
  for(int i = 0; i < static_cast<int>(visible_sample_points_.size()); i++)
  {
    double dx = visible_sample_points_[i].dx;
    double dy = visible_sample_points_[i].dy;

    int x = static_cast<int>(visible_sample_points_[i].coord2.x);
    int y = static_cast<int>(visible_sample_points_[i].coord2.y);

    // avoid invisible 2D points
    if(x < 0 || x >= width_)
      continue;
    if(y < 0 || y >= height_)
      continue;

    visible_sample_points_[i].dist = maxd_;
    visible_sample_points_[i].nuv = cvPoint2D32f(dx, dy);

    for(int j = 0; j < max_step; j++)
    {
      // positive direction
      x = round(visible_sample_points_[i].coord2.x + dx*(double)j);
      y = round(visible_sample_points_[i].coord2.y + dy*(double)j);
      // determine the new indices are valid or not
      if(x >= 0 && x < width_ && y >= 0 && y < height_ && *(img_edge_->imageData + (x) + (y)*img_edge_->widthStep) == char(255)) // find edge
      {
        const float* ptr_gx = (const float*)(img_gx_->imageData + y*img_gx_->widthStep);
        const float* ptr_gy = (const float*)(img_gy_->imageData + y*img_gy_->widthStep);
        float ori = atan2(ptr_gy[x], ptr_gx[x])*180.f/M_PI;
        if(_withinOri(ori, visible_sample_points_[i].normal_ang_deg, oth)) // check orientation
        {
          visible_sample_points_[i].dist = sqrt(double(j)*dx*double(j)*dx + double(j)*dy*double(j)*dy);
          visible_sample_points_[i].edge_pt2 = cvPoint2D32f(static_cast<float>(x), static_cast<float>(y));
          break;
        }
      }

      if(j == 0) // skip (already did)
        continue;

      // negative direction
      x = round(visible_sample_points_[i].coord2.x - dx*(double)j);
      y = round(visible_sample_points_[i].coord2.y - dy*(double)j);
      // determine the new indices are valid or not
      if(x >= 0 && x < width_ && y >= 0 && y < height_ && *(img_edge_->imageData + (x) + (y)*img_edge_->widthStep) == char(255)) // find edge
      {
        const float* ptr_gx = (const float*)(img_gx_->imageData + y*img_gx_->widthStep);
        const float* ptr_gy = (const float*)(img_gy_->imageData + y*img_gy_->widthStep);
        float ori = atan2(ptr_gy[x], ptr_gx[x])*180.f/M_PI;
        if(_withinOri(ori, visible_sample_points_[i].normal_ang_deg, oth)) // check orientation
        {
          visible_sample_points_[i].dist = sqrt(double(j)*dx*double(j)*dx + double(j)*dy*double(j)*dy);
          visible_sample_points_[i].edge_pt2 = cvPoint2D32f(static_cast<float>(x), static_cast<float>(y));
          // flip direction
          visible_sample_points_[i].nuv.x = -1*visible_sample_points_[i].nuv.x;
          visible_sample_points_[i].nuv.y = -1*visible_sample_points_[i].nuv.y;
          break;
        }
      }
    }
  }
}

int CObjectModel::refineEdgeCorrespondences_RANSAC(CvMat *E, int N/*=1000*/, double th/*=10.0*/)
{
  // E: [4 x 4] current extrinsic matrix (pose)
  // N: number of iterations
  vector<int> valid_idx;
  valid_idx.clear();
  //valid_idx.resize(visible_sample_points_.size());
  for(int i=0; i<visible_sample_points_.size(); i++)
  {
    if(visible_sample_points_[i].dist < maxd_)
    {
      valid_idx.push_back(i);
    }
  }

  double iter = 0.0; // iteration number
  const int nom = 6; // minimum number of points required
  double k = 100000.00;
  const double p = 0.99;
  int nop = valid_idx.size();
  if(nop < nom)
  {
    cout << "Insufficient num of points in RANSAC ..." << endl;
    return -1;
  }

  int best_noi = 0;
  CvRNG rng = cvRNG(cvGetTickCount());
  int rand_idx[nom];
  vector<SamplePoint> sp;
  sp.resize(nom);
  vector<int> inlier_idx;
  inlier_idx.resize(nop);
  vector<int> best_inlier_idx;
  best_inlier_idx.resize(nop);


  CvMat* P = cvCreateMat(3,4,CV_32F);
  CvMat* P2 = cvCreateMat(3,4,CV_32F);
  CvMat* x3d_h = cvCreateMat(4, nop, CV_32F);
  CvMat* x2d_proj = cvCreateMat(3, nop, CV_32F);
  CvMat* pose_cur = cvCreateMat(4, 4, CV_32F);

  for(int i=0; i<nop; i++)
  {
    CV_MAT_ELEM(*x3d_h, float, 0, i) = visible_sample_points_[valid_idx[i]].coord3.x;
    CV_MAT_ELEM(*x3d_h, float, 1, i) = visible_sample_points_[valid_idx[i]].coord3.y;
    CV_MAT_ELEM(*x3d_h, float, 2, i) = visible_sample_points_[valid_idx[i]].coord3.z;
    CV_MAT_ELEM(*x3d_h, float, 3, i) = 1.0;
  }

  while(iter < k && iter < double(N))
  {
    // random sampling
    for(int i=0; i<nom; i++)
    {
      int temp_idx = 0;
      bool found = true;
      while(found)
      {
        temp_idx = cvRandInt(&rng) % nop;
        found = false;
        for(int j=0; j<i; j++)
        {
          if(rand_idx[j] == temp_idx)
            found = true;
        }
      }
      rand_idx[i] = temp_idx;
    }
    // estimate pose
    for(int i=0; i<nom; i++)
    {
      sp[i] = visible_sample_points_[valid_idx[rand_idx[i]]];
    }
    //CvMat* Mcur = edge_tracker_->getEstimatedPoseLS(*E, sp);
    edge_tracker_->getEstimatedPoseIRLS(pose_cur, E, sp);
    for(int r=0; r<3; r++)
      for(int c=0; c<4; c++)
        //CV_MAT_ELEM(*P, float, r, c) = CV_MAT_ELEM(*Mcur, float, r, c);
        CV_MAT_ELEM(*P, float, r, c) = CV_MAT_ELEM(*E, float, r, c);
    cvGEMM(intrinsic_, P, 1, NULL, 0, P2, 0);
    //cvReleaseMat(&Mcur);

    // x2d_proj = P * x3d_h
    cvGEMM(P2, x3d_h, 1, NULL, 0, x2d_proj, 0);

    for(int i=0; i<nop; i++)
    {
      float u = CV_MAT_ELEM(*x2d_proj, float, 0, i);
      float v = CV_MAT_ELEM(*x2d_proj, float, 1, i);
      float w = CV_MAT_ELEM(*x2d_proj, float, 2, i);

      CV_MAT_ELEM(*x2d_proj, float, 0, i) = u/w;
      CV_MAT_ELEM(*x2d_proj, float, 1, i) = v/w;
      // save reprojection error to third rows
      CV_MAT_ELEM(*x2d_proj, float, 2, i) = sqrt(
        (CV_MAT_ELEM(*x2d_proj, float, 0, i) - visible_sample_points_[valid_idx[i]].edge_pt2.x)
        *(CV_MAT_ELEM(*x2d_proj, float, 0, i) - visible_sample_points_[valid_idx[i]].edge_pt2.x) + 
        (CV_MAT_ELEM(*x2d_proj, float, 1, i) - visible_sample_points_[valid_idx[i]].edge_pt2.y)
        *(CV_MAT_ELEM(*x2d_proj, float, 1, i) - visible_sample_points_[valid_idx[i]].edge_pt2.y));
    }

    // Count number of inliers
    int noi = 0;
    for(int i=0; i<nop; i++) 
    {
      if(CV_MAT_ELEM(*x2d_proj, float, 2, i)  < th)
      {
        inlier_idx[i] = 1;
        noi++;
      }
      else
        inlier_idx[i] = 0;
    }

    if(noi > best_noi) 
    {
      best_noi = noi;
      best_inlier_idx = inlier_idx;
      // Determine adaptive number of iteration
      double e = 1. - (double)best_noi/(double)nop;
      k = (double)(log(1. - p)/log(1. - pow(1.-e, nom)));
    }

    iter++;
  }
  // remove outliers
  for(int i=0; i<nop; i++)
  {
    if(best_inlier_idx[i] == 0)
    {
      assert(visible_sample_points_[valid_idx[i]].dist < maxd_);
      visible_sample_points_[valid_idx[i]].dist = maxd_; // maxd_ distance sample points will be ignored
    }
  }

  int noi2 = 0;
  for(int i=0; i<visible_sample_points_.size(); i++)
  {
    if(visible_sample_points_[i].dist < maxd_)
      noi2++;
  }

  assert(best_noi == noi2);

  cvReleaseMat(&P);
  cvReleaseMat(&P2);
  cvReleaseMat(&x3d_h);
  cvReleaseMat(&x2d_proj);
  cvReleaseMat(&pose_cur);

  return int(iter);
}

void CObjectModel::determineSharpEdges(GLMmodel* model, float th_sharp, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& sharp_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_normals)
{
  // Determine sharp edges
  for(int i=0; i<model->numvertices-1; i++)
  {
    printf("Determine sharp edges: %d/%d ...\n", i+1, model->numvertices);
    // find faces containing current vertex
    std::vector<std::pair<int, std::pair<int, int> > > vertex_association; // (refered vertex, (face index 1, face index 2))
    for(int k=0; k<model->numtriangles; k++)
    {
      // vindices are one-based
      int vi[3];
      vi[0] = model->triangles[k].vindices[0] - 1;
      vi[1] = model->triangles[k].vindices[1] - 1;
      vi[2] = model->triangles[k].vindices[2] - 1;

      if((i == vi[0]) || (i == vi[1]) || (i == vi[2])) // found 'i' vertex?
      {
        for(int vii=0; vii<3; vii++)
        {
          if(vi[vii] > i) // new vertex index shouldn't be i nor smaller than i (already considered before)
          {
            // check if current refered vertex is already in 'vertex_assocation'
            bool found = false;
            for(int l=0; l<vertex_association.size(); l++)
            {
              if(vertex_association[l].first == vi[vii])
              {
                found = true;
                assert(vertex_association[l].second.second != k);
                if(vertex_association[l].second.second == -1)
                {
                  vertex_association[l].second.second = k;
                }
                else
                {
                  std::cout << "found a duplicated face (skip this face)" << std::endl;
                }
                break;
              }
            }
            if(!found)
            {
              std::pair<int, std::pair<int, int> > va;
              va.first = vi[vii]; // refered vetex index
              va.second.first = k; // first face index
              va.second.second = -1; // second face index
              vertex_association.push_back(va);
            }
          }
        }
      }
    }

    for(int l=0; l<vertex_association.size(); l++)
    {
      assert(vertex_association[l].first > i);
      assert(vertex_association[l].second.first != -1);
      if(vertex_association[l].second.second != -1) // found two triangles
      {
        int tri1 = vertex_association[l].second.first;
        int tri2 = vertex_association[l].second.second;
        int j = vertex_association[l].first;
        // Check the sharpness by calculating an inner product of two face normals
        float x1 = model->facetnorms[3*(tri1+1)+0];
        float y1 = model->facetnorms[3*(tri1+1)+1];
        float z1 = model->facetnorms[3*(tri1+1)+2];

        float x2 = model->facetnorms[3*(tri2+1)+0];
        float y2 = model->facetnorms[3*(tri2+1)+1];
        float z2 = model->facetnorms[3*(tri2+1)+2];

        float inner_prod = x1*x2 + y1*y2 + z1*z2;

        // Inner product results should be close to 0.
        if(-th_sharp <= inner_prod && inner_prod <= th_sharp)
        {
          // It is a sharp edge (i)-(j)
          sharp_edges.push_back(
            std::pair<CvPoint3D32f, CvPoint3D32f>(
            cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2]),
            cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2])
            )
          );
        }
        else
        {
          // First triangle
          int v10 = model->triangles[tri1].vindices[0] - 1;
          int v11 = model->triangles[tri1].vindices[1] - 1;
          int v12 = model->triangles[tri1].vindices[2] - 1;
          // It is an dull edge (i)-(j)
          // meet counter clockwise convenction
          if((v10 == i && v11 == j) || (v11 == i && v12 == j) || (v10 == j && v12 == i))
          {
            dull_edges.push_back(
              std::pair<CvPoint3D32f, CvPoint3D32f>(
              cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2]),
              cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2])
              )
            );
          }
          else
          {
            dull_edges.push_back(
              std::pair<CvPoint3D32f, CvPoint3D32f>(
              cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2]),
              cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2])
              )
            );
          }
          // save left one point
          int v1_left;
          if(v10 != i && v10 != j)
            v1_left = v10 + 1;
          else if(v11 != i && v11 != j)
            v1_left = v11 + 1;
          else
          {
            assert(v12 != i && v12 != j);
            v1_left = v12 + 1;
          }

          // Second triangle
          int v20 = model->triangles[tri2].vindices[0] - 1;
          int v21 = model->triangles[tri2].vindices[1] - 1;
          int v22 = model->triangles[tri2].vindices[2] - 1;
          // save left one point
          int v2_left;
          if(v20 != i && v20 != j)
            v2_left = v20 + 1;
          else if(v21 != i && v21 != j)
            v2_left = v21 + 1;
          else
          {
            assert(v22 != i && v22 != j);
            v2_left = v22 + 1;
          }

          dull_normals.push_back(
            std::pair<CvPoint3D32f, CvPoint3D32f>(
            cvPoint3D32f(model->vertices[3*v1_left+0], model->vertices[3*v1_left+1], model->vertices[3*v1_left+2]),
            cvPoint3D32f(model->vertices[3*v2_left+0], model->vertices[3*v2_left+1], model->vertices[3*v2_left+2])
            )
          );
        }
      }
#if 1
      if(vertex_association[l].second.second == -1) // found only one triangle. some model need this edge as boundary
      {
        int j = vertex_association[l].first;
        sharp_edges.push_back(
          std::pair<CvPoint3D32f, CvPoint3D32f>(
          cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2]),
          cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2])
          )
        );
      }
#endif
    }
  }
}

void CObjectModel::determineSharpEdges_slow(GLMmodel* model, float th_sharp, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& sharp_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_normals)
{
  // Determine sharp edges
  for(int i=0; i<model->numvertices-1; i++)
  {
    printf("Determine sharp edges: %d/%d ...\n", i+1, model->numvertices);
    for(int j=i+1; j<model->numvertices; j++)
    {
      // i, j: indices of two vertices (zero-based)

      // Find two triangles sharing (i)-(j) edge
      int tri1 = -1, tri2 = -1;
      for(int k=0; k<model->numtriangles; k++)
      {
        // vindices are one-based
        int v1 = model->triangles[k].vindices[0] - 1;
        int v2 = model->triangles[k].vindices[1] - 1;
        int v3 = model->triangles[k].vindices[2] - 1;                
        if((v1 == i && v2 == j) || (v1 == j && v2 == i) || (v2 == i && v3 == j) || (v2 == j && v3 == i) || (v3 == i && v1 == j) || (v3 == j && v1 == i))
        {
          if(tri1 == -1)
          {
            tri1 = k;
          }
          else
          {
            tri2 = k;
            break; // There is only one edge that two triangles share
          }
        }
      }
      // If it finds two triangles
      if(tri1 != -1 && tri2 != -1)
      {
        // Check the sharpness by calculating an inner product of two face normals
        float x1 = model->facetnorms[3*(tri1+1)+0];
        float y1 = model->facetnorms[3*(tri1+1)+1];
        float z1 = model->facetnorms[3*(tri1+1)+2];

        float x2 = model->facetnorms[3*(tri2+1)+0];
        float y2 = model->facetnorms[3*(tri2+1)+1];
        float z2 = model->facetnorms[3*(tri2+1)+2];

        float inner_prod = x1*x2 + y1*y2 + z1*z2;

        // Inner product results should be close to 0.
        if(-th_sharp <= inner_prod && inner_prod <= th_sharp)
        {
          // It is a sharp edge (i)-(j)
          sharp_edges.push_back(
            std::pair<CvPoint3D32f, CvPoint3D32f>(
            cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2]),
            cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2])
            )
          );
        }
        else
        {
          // First triangle
          int v10 = model->triangles[tri1].vindices[0] - 1;
          int v11 = model->triangles[tri1].vindices[1] - 1;
          int v12 = model->triangles[tri1].vindices[2] - 1;
          // It is an dull edge (i)-(j)
          // meet counter clockwise convenction
          if((v10 == i && v11 == j) || (v11 == i && v12 == j) || (v10 == j && v12 == i))
          {
            dull_edges.push_back(
              std::pair<CvPoint3D32f, CvPoint3D32f>(
              cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2]),
              cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2])
              )
              );
          }
          else
          {
            dull_edges.push_back(
              std::pair<CvPoint3D32f, CvPoint3D32f>(
              cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2]),
              cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2])
              )
              );
          }
          // save left one point
          int v1_left;
          if(v10 != i && v10 != j)
            v1_left = v10 + 1;
          else if(v11 != i && v11 != j)
            v1_left = v11 + 1;
          else
          {
            assert(v12 != i && v12 != j);
            v1_left = v12 + 1;
          }

          // Second triangle
          int v20 = model->triangles[tri2].vindices[0] - 1;
          int v21 = model->triangles[tri2].vindices[1] - 1;
          int v22 = model->triangles[tri2].vindices[2] - 1;
          // save left one point
          int v2_left;
          if(v20 != i && v20 != j)
            v2_left = v20 + 1;
          else if(v21 != i && v21 != j)
            v2_left = v21 + 1;
          else
          {
            assert(v22 != i && v22 != j);
            v2_left = v22 + 1;
          }

          dull_normals.push_back(
            std::pair<CvPoint3D32f, CvPoint3D32f>(
            cvPoint3D32f(model->vertices[3*v1_left+0], model->vertices[3*v1_left+1], model->vertices[3*v1_left+2]),
            cvPoint3D32f(model->vertices[3*v2_left+0], model->vertices[3*v2_left+1], model->vertices[3*v2_left+2])
            )
            );
        }
      }
#if 1
      // If it finds one triangle, still need it. (boundary)
      // Some model might need it
      else if(tri1 != -1 && tri2 == -1)
      {
        // It is a sharp edge (i)-(j)
        sharp_edges.push_back(
          std::pair<CvPoint3D32f, CvPoint3D32f>(
          cvPoint3D32f(model->vertices[3*(i+1)+0], model->vertices[3*(i+1)+1], model->vertices[3*(i+1)+2]),
          cvPoint3D32f(model->vertices[3*(j+1)+0], model->vertices[3*(j+1)+1], model->vertices[3*(j+1)+2])
          )
          );
      }
#endif
    }
  }
}

void CObjectModel::loadObjectCADModel(string obj_name)
{
  // Load CAD model from obj file & generate a display list for fast drawing

  if(meshmodel_ != NULL)
    free(meshmodel_);

  meshmodel_ = glmReadOBJ((char *)(obj_name + std::string(".obj")).c_str());

  if(!meshmodel_)
  {
    std::cerr << "Error in reading OBJ file" << std::endl;
    exit(-1);
  }
  glmFacetNormals(meshmodel_);          // (re)calculate face normals
  glmVertexNormals(meshmodel_, 90.0);   // (re)calculate vertex normals

  // Load or generate sampling points along the sharp edges

  // load sharp edges (tracking edges) if exist
  CvMat* Mobj = (CvMat*)cvLoad((obj_name + std::string(".xml")).c_str());
  if(Mobj)
  {
    // matrix dimension: (2 * num of edges) by 3
    sharp_edges_.clear();
    dull_edges_.clear();
    dull_normals_.clear();

    int size_vSEP = (int)CV_MAT_ELEM(*Mobj, float, 0, 0);
    int size_vDEP = (int)CV_MAT_ELEM(*Mobj, float, 0, 1);
    int size_vDEN = (int)CV_MAT_ELEM(*Mobj, float, 0, 2);
    int start_idx = 1;
    
    sharp_edges_.resize(size_vSEP);
    dull_edges_.resize(size_vDEP);
    dull_normals_.resize(size_vDEN);

    for(int i=0; i<size_vSEP; i++)      sharp_edges_[i].first   = getPoint(Mobj, i + start_idx);
    start_idx += size_vSEP;
    for(int i=0; i<size_vSEP; i++)      sharp_edges_[i].second  = getPoint(Mobj, i + start_idx);
    start_idx += size_vSEP;
    for(int i=0; i<size_vDEP; i++)      dull_edges_[i].first    = getPoint(Mobj, i + start_idx);
    start_idx += size_vDEP;
    for(int i=0; i<size_vDEP; i++)      dull_edges_[i].second   = getPoint(Mobj, i + start_idx);
    start_idx += size_vDEP;
    for(int i=0; i<size_vDEN; i++)      dull_normals_[i].first  = getPoint(Mobj, i + start_idx);
    start_idx += size_vDEN;
    for(int i=0; i<size_vDEN; i++)      dull_normals_[i].second = getPoint(Mobj, i + start_idx);
  }
  else // if it fails to load xml file, calculate and make xml file
  {
    determineSharpEdges(meshmodel_, th_sharp_, sharp_edges_, dull_edges_, dull_normals_);

    // Save the tracking edges into xml file
    // first row is for setting size of each
    int row_len = 1 + 2*sharp_edges_.size() + 2*dull_edges_.size() + 2*dull_normals_.size(); 
    float* f_buf = (float *)malloc(sizeof(float)*(row_len)*3);
    CvMat A = cvMat(row_len, 3, CV_32F, f_buf);
    CV_MAT_ELEM(A, float, 0, 0) = (float)sharp_edges_.size();
    CV_MAT_ELEM(A, float, 0, 1) = (float)dull_edges_.size();
    CV_MAT_ELEM(A, float, 0, 2) = (float)dull_normals_.size();
    int start_idx = 1;
    for(int i=0; i<sharp_edges_.size(); i++)      setPoint(&A, i + start_idx, sharp_edges_[i].first);
    start_idx += sharp_edges_.size();    
    for(int i=0; i < sharp_edges_.size(); i++)    setPoint(&A, i + start_idx, sharp_edges_[i].second);
    start_idx += sharp_edges_.size();
    for(int i=0; i<dull_edges_.size(); i++)       setPoint(&A, i + start_idx, dull_edges_[i].first);
    start_idx += dull_edges_.size();
    for(int i=0; i<dull_edges_.size(); i++)       setPoint(&A, i + start_idx, dull_edges_[i].second);
    start_idx += dull_edges_.size();
    for(int i=0; i<dull_normals_.size(); i++)     setPoint(&A, i + start_idx, dull_normals_[i].first);
    start_idx += dull_normals_.size();
    for(int i=0; i<dull_normals_.size(); i++)     setPoint(&A, i + start_idx, dull_normals_[i].second);
    cvSave((obj_name + std::string(".xml")).c_str(), &A);
    free(f_buf);
  }
  
  generateSamplePoints(sharp_sample_points_, sharp_sample_points_edge_indices_, sharp_edges_);
  
  if(dulledge_)
  {
    // generate sampling points on unsharpened edges
    // that will be tested for silhouette
    set_dull_sample_points_.clear();
    set_dull_sample_points_.resize(dull_edges_.size());
    for(int i=0; i<dull_edges_.size(); i++)
    {
      // get an unit vector of displacement between m_vDEP1 and m_vDEP2
      CvPoint3D32f dis = cvPoint3D32f(
        dull_edges_[i].second.x - dull_edges_[i].first.x,
        dull_edges_[i].second.y - dull_edges_[i].first.y,
        dull_edges_[i].second.z - dull_edges_[i].first.z
      );

      double length = sqrt(dis.x*dis.x + dis.y*dis.y + dis.z*dis.z);
      CvPoint3D32f u = cvPoint3D32f(dis.x/float(length), dis.y/float(length), dis.z/float(length));
      // determine each step in x, y, and z direction
      CvPoint3D32f step = cvPoint3D32f(u.x*sample_step_, u.y*sample_step_, u.z*sample_step_);
      // generate sample points
      for(float j = (include_starting_point_ ? 0.f : 1.f); j< float(length)/sample_step_; j++)
        set_dull_sample_points_[i].push_back(
          cvPoint3D32f(
            dull_edges_[i].first.x + step.x*j, 
            dull_edges_[i].first.y + step.y*j, 
            dull_edges_[i].first.z + step.z*j
          )
        );
    }
  }

  // generate a display list
  dl_ = glGenLists(1);
  if(!dl_) 
  {
    std::cerr << "Fail to create a display list." << std::endl;
    return; 
  }

  // store drawing function in the display list
  glNewList(dl_, GL_COMPILE);
  for(int i=0; i<meshmodel_->numtriangles; i++)
  {
    int v1 = meshmodel_->triangles[i].vindices[0];
    int v2 = meshmodel_->triangles[i].vindices[1];
    int v3 = meshmodel_->triangles[i].vindices[2];

    float x1 = meshmodel_->vertices[3*v1+0];
    float y1 = meshmodel_->vertices[3*v1+1];
    float z1 = meshmodel_->vertices[3*v1+2];

    float x2 = meshmodel_->vertices[3*v2+0];
    float y2 = meshmodel_->vertices[3*v2+1];
    float z2 = meshmodel_->vertices[3*v2+2];

    float x3 = meshmodel_->vertices[3*v3+0];
    float y3 = meshmodel_->vertices[3*v3+1];
    float z3 = meshmodel_->vertices[3*v3+2];

    glBegin(GL_POLYGON);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glVertex3f(x3, y3, z3);
    glEnd();
  }
  glEndList();
}

void CObjectModel::generateSamplePoints(std::vector<CvPoint3D32f>& sample_points, std::vector<int>& edge_index, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& edges)
{
  sample_points.clear();
  edge_index.clear();

  // generate control points (red points) 
  for(size_t i = 0; i < edges.size(); i++)
  {
    // get an unit vector of displacement between starting and end points of an edge
    CvPoint3D32f dis = cvPoint3D32f(
      edges[i].second.x - edges[i].first.x,
      edges[i].second.y - edges[i].first.y,
      edges[i].second.z - edges[i].first.z
      );
    double length = sqrt(dis.x*dis.x + dis.y*dis.y + dis.z*dis.z);
    CvPoint3D32f u = cvPoint3D32f(dis.x/float(length), dis.y/float(length), dis.z/float(length));
    // determine each step in x, y, and z direction
    CvPoint3D32f step = cvPoint3D32f(u.x*sample_step_, u.y*sample_step_, u.z*sample_step_);
    // generate sample points
    for(float j = (include_starting_point_ ? 0.f : 1.f); j< float(length)/sample_step_; j++)
    {
      sample_points.push_back(
        cvPoint3D32f(
          edges[i].first.x + step.x*j, 
          edges[i].first.y + step.y*j, 
          edges[i].first.z + step.z*j
        )
      );
      edge_index.push_back(i);
    }
  }
}

float CObjectModel::getTriangleArea(GLMmodel* m, int tri_idx)
{
  // Calculate the area of triangle
  int vi1 = m->triangles[tri_idx].vindices[0];
  int vi2 = m->triangles[tri_idx].vindices[1];
  int vi3 = m->triangles[tri_idx].vindices[2];

  float vc1x = meshmodel_->vertices[3*(vi1)+0];
  float vc1y = meshmodel_->vertices[3*(vi1)+1];
  float vc1z = meshmodel_->vertices[3*(vi1)+2];

  float vc2x = meshmodel_->vertices[3*(vi2)+0];
  float vc2y = meshmodel_->vertices[3*(vi2)+1];
  float vc2z = meshmodel_->vertices[3*(vi2)+2];

  float vc3x = meshmodel_->vertices[3*(vi3)+0];
  float vc3y = meshmodel_->vertices[3*(vi3)+1];
  float vc3z = meshmodel_->vertices[3*(vi3)+2];

  float a_dx = vc2x - vc1x;
  float a_dy = vc2y - vc1y;
  float a_dz = vc2z - vc1z;

  float b_dx = vc3x - vc1x;
  float b_dy = vc3y - vc1y;
  float b_dz = vc3z - vc1z;

  float area = (a_dx*b_dx + a_dy*b_dy + a_dz*b_dz)/2.0f;
  return area;
}

bool CObjectModel::isEnoughValidSamplePoints(double th_ratio/*=0.5*/)
{
  // Check # of visible sample points are enough
  // return 0: not enough
  // return 1: enough
  int count = 0;

  for(int i=0; i<int(visible_sample_points_.size()); i++)
    if(visible_sample_points_[i].dist < maxd_) // only valid points
      count++;

  if(double(count) >= th_ratio*double(visible_sample_points_.size()))
    return true;
  else
    return false;
}

double CObjectModel::GetValidVisibleSamplePointsRatio()
{
  // return ratio of visible sample points
  int count = 0;
  for(size_t i = 0; i < visible_sample_points_.size(); i++)
    if(visible_sample_points_[i].dist < maxd_) // only valid points
      count++;

  return (double)count/(double)visible_sample_points_.size();
}

void CObjectModel::displayPoseLine(IplImage* img_dest, CvMat* pose/*=NULL*/, CvScalar color/*=CV_RGB(255, 255, 0)*/, int thickness/*=1*/, bool onright/*= false*/)
{
  // if pose is not set, use current pose estimate (pose_)
  if(!pose)
    pose = pose_;

  float inf = numeric_limits<float>::infinity();

  // draw wireframe model with lines
  for(int i=0; i<sharp_edges_.size(); i++)
  {
    CvPoint2D32f pt21 = project3Dto2D(sharp_edges_[i].first, pose);
    CvPoint2D32f pt22 = project3Dto2D(sharp_edges_[i].second, pose);

    if(onright) // pose init results may require drawing on right image
    {
      pt21.x += static_cast<float>(img_dest->width/2);
      pt22.x += static_cast<float>(img_dest->width/2);
    }

    if(pt21.x != inf && pt21.y != inf && pt22.x != inf && pt22.y != inf)
      cvLine(img_dest, cvPointFrom32f(pt21), cvPointFrom32f(pt22), color, thickness, draw_type_);
  }

  if(dulledge_)
  {
    // draw dull edges as well
    for(int i=0; i<dull_boundary_edges_.size(); i++)
    {
      CvPoint2D32f pt21 = project3Dto2D(dull_boundary_edges_[i].first, pose);
      CvPoint2D32f pt22 = project3Dto2D(dull_boundary_edges_[i].second, pose);

      if(onright) // pose init results may require drawing on right image
      {
        pt21.x += static_cast<float>(img_dest->width/2);
        pt22.x += static_cast<float>(img_dest->width/2);
      }

      if(pt21.x != inf && pt21.y != inf && pt22.x != inf && pt22.y != inf)
        cvLine(img_dest, cvPointFrom32f(pt21), cvPointFrom32f(pt22), color, thickness, draw_type_);
    }
  }
}

void CObjectModel::displaySamplePointsAndErrors(IplImage* img_dest)
{
  // Show visible edge sample points and their errors
  if(use_fine_orientation_)
    drawPointsAndErrorFineOri(img_dest);
  else
    drawPointsAndErrorCoarseOri(img_dest);
}
