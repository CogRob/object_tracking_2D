#include <string>

//#include "tracker_irls.h"
//#include "tracker_pf_texture.h"
//#include "tracker_pf_textureless.h"

#include <boost/signals2/mutex.hpp> 
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>        // openmp
#include <iostream>
//#include <direct.h> // mkdir(), creating a directory




#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <limits.h>

#include <ObjectModel.h>

#include <Fitline/LFLineFitter.h>

namespace po = boost::program_options;

CvPoint2D32f project3Dto2D(CvPoint3D32f pt3, CvMat* pose, CvMat* param_intrinsic)
{
  CvPoint2D32f pt2;
  CvPoint3D32f pt3_cam;
  pt3_cam.x = CV_MAT_ELEM(*pose, float, 0, 0)*pt3.x + CV_MAT_ELEM(*pose, float, 0, 1)*pt3.y + CV_MAT_ELEM(*pose, float, 0, 2)*pt3.z + CV_MAT_ELEM(*pose, float, 0, 3);
  pt3_cam.y = CV_MAT_ELEM(*pose, float, 1, 0)*pt3.x + CV_MAT_ELEM(*pose, float, 1, 1)*pt3.y + CV_MAT_ELEM(*pose, float, 1, 2)*pt3.z + CV_MAT_ELEM(*pose, float, 1, 3);
  pt3_cam.z = CV_MAT_ELEM(*pose, float, 2, 0)*pt3.x + CV_MAT_ELEM(*pose, float, 2, 1)*pt3.y + CV_MAT_ELEM(*pose, float, 2, 2)*pt3.z + CV_MAT_ELEM(*pose, float, 2, 3);

  float fx = CV_MAT_ELEM(*param_intrinsic, float, 0, 0);
  float fy = CV_MAT_ELEM(*param_intrinsic, float, 1, 1);
  float ux = CV_MAT_ELEM(*param_intrinsic, float, 0, 2);
  float uy = CV_MAT_ELEM(*param_intrinsic, float, 1, 2);

  pt2.x = fx*pt3_cam.x/pt3_cam.z + ux;
  pt2.y = fy*pt3_cam.y/pt3_cam.z + uy;

  float th_d = 10000.0;
  if(pt2.x < -th_d || pt2.x > th_d || pt2.y < -th_d || pt2.y > th_d)
  {
    pt2.x = -1;
    pt2.y = -1;
  }

  return pt2;
}

CvRect drawModel(IplImage* img, std::vector<CvPoint3D32f> ep1, std::vector<CvPoint3D32f> ep2, CvMat* pose, CvMat* param_intrinsic, CvScalar color)
{
  float widthf = static_cast<float>(img->width), heightf = static_cast<float>(img->height);
  CvPoint2D32f pt21, pt22;
  float l = numeric_limits<float>::max(), r = 0.0f, t = numeric_limits<float>::max(), b = 0.0f;
  for(int i=0; i<ep1.size(); i++)
  {
    pt21 = project3Dto2D(ep1[i], pose, param_intrinsic);
    pt22 = project3Dto2D(ep2[i], pose, param_intrinsic);
    cvLine(img, cvPointFrom32f(pt21), cvPointFrom32f(pt22), color, 1, 8);
    if(pt21.x < l) l = pt21.x;
    if(pt21.x > r) r = pt21.x;
    if(pt21.y < t) t = pt21.y;
    if(pt21.y > b) b = pt21.y;

    if(pt22.x < l) l = pt22.x;
    if(pt22.x > r) r = pt22.x;
    if(pt22.y < t) t = pt22.y;
    if(pt22.y > b) b = pt22.y;
  }

  l = max(0.f, l);
  r = min(widthf - 1.f, r);
  t = max(0.f, t);
  b = min(heightf - 1.f, b);
  return cvRect(static_cast<int>(l), static_cast<int>(t), static_cast<int>(r-l+1.f), static_cast<int>(b-t+1.f));
}

int main(int argc, char **argv)
{
  std::string obj_name;
  std::string intrinsic;
  int width;
  int height;
  float sample_step;
  bool dull_edge;
  std::string str_result_path;
  std::string str_param_linfit;
  float depth;

  po::options_description desc("\nTemplate generator keys:\nk: save current template\nw: translate further\ns: translate closer\nq,e,a,d,z,d: rotates in 3 axes\n\nTemplate generator options");
  desc.add_options()
    ("help,h", "produce help message")
    ("obj-name,o", po::value<std::string>(&obj_name), "name of traget object")
    ("sample-step,s", po::value<float>(&sample_step)->default_value(0.005f), "sample step")
    ("depth,d", po::value<float>(&depth)->default_value(0.3f), "distance between object and camera")
    ("param-linefit,l", po::value<std::string>(&str_param_linfit)->default_value(std::string("para_template_line_fitter.txt")), "set parameters for line fitting")
    ("save-path,p", po::value<std::string>(&str_result_path)->default_value(std::string("data_newobject")), "set result path")
    
    ("width", po::value<int>(&width)->default_value(640), "width")
    ("height", po::value<int>(&height)->default_value(480), "height")
    ("intrinsic", po::value<std::string>(&intrinsic)->default_value("Intrinsics_normal.xml"), "intrinsic parameters")
    ("dull_edge", po::value<bool>(&dull_edge)->default_value(true), "consider dull edges")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(argc < 2 || vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  if(obj_name.empty())
  {
    std::cerr << "obj-name should be specified." << std::endl;
    return 1;
  }

  if(vm.count("obj-name"))
  {
    std::cout << "obj-name: " << vm["obj-name"].as<std::string>() << std::endl;
  }
 
  if(vm.count("save-path"))
  {
    if(mkdir(str_result_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1 && errno != EEXIST)
    {
      std::cerr << "Cannot create " << str_result_path << " directory for saving results." << std::endl;
      return false;
    }
  }

  IplImage* img_result = cvCreateImage(cvSize(width, height), 8, 1);
  CvMat* pose = cvCreateMat(4, 4, CV_32F);
  char key = 0;
  float modelPosition[3];
  float modelAngle[3];
  float matrixModel[16];
  modelPosition[0] = 0.0;
  modelPosition[1] = 0.0;
  modelPosition[2] = depth;
  modelAngle[0] = 0.0;
  modelAngle[1] = 0.0;
  modelAngle[2] = 0.0;
  int int_not = 0; // number of templates

  CvMat* param_intrinsic = (CvMat*)cvLoad(intrinsic.c_str());

  // Create object model instance
  int maxd = 16;
  CObjectModel cObjModel(obj_name, width, height, param_intrinsic, sample_step, maxd, dull_edge, NULL);

  cvSetIdentity(pose);
  CV_MAT_ELEM(*pose, float, 2, 3) = depth;

  // init line fitter
  LFLineFitter lf;
  lf.Init();
  lf.Configure("para_template_line_fitter.txt");

  cvNamedWindow("Edge");

  while(key != 27) // until 'esc'
  {
    cvSet(img_result, cvScalar(0)); // reset image

    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    // In order to get the modeling matrix only, reset GL_MODELVIEW matrix
    glLoadIdentity();
    // transform the object
    // From now, all transform will be for modeling matrix only. (transform from object space to world space)
    glTranslatef(modelPosition[0], modelPosition[1], modelPosition[2]);
    glRotatef(modelAngle[0], 1, 0, 0);
    glRotatef(modelAngle[1], 0, 1, 0);
    glRotatef(modelAngle[2], 0, 0, 1);
    // save modeling matrix
    glGetFloatv(GL_MODELVIEW_MATRIX, matrixModel);

    CvMat* poset = cvCreateMat(4, 4, CV_32F);
    memcpy(poset->data.fl, matrixModel, sizeof(float)*16);
    cvTranspose(poset, pose);
    cvReleaseMat(&poset);
    glPopMatrix();

    // Draw object model with visibility test
    cObjModel.setModelviewMatrix(pose); // update the initial pose to object model for displaying    
    cObjModel.findVisibleSamplePoints(); // draw object model with visibility test

    // Find visible edges
    std::vector<CvPoint3D32f> ep1, ep2;
   
    std::vector<CObjectModel::SamplePoint>& vsp = cObjModel.getVisibleSamplePoints();
    
    // determine two end points in each common edge_mem sample points
    int edge_mem = vsp[0].edge_mem;
    ep1.push_back(vsp[0].coord3);
    int i;
    for(i=0; i<int(vsp.size()); i++)
    {
      if(edge_mem == vsp[i].edge_mem)
      {
        // skip over
      }
      else
      {
        // new point, so add end/starting edge point
        ep2.push_back(vsp[i-1].coord3);
        ep1.push_back(vsp[i].coord3);
        // update new edge_mem value
        edge_mem = vsp[i].edge_mem;
      }
    }
    ep2.push_back(vsp[i-1].coord3);

    
    CvRect bound = drawModel(img_result, ep1, ep2, pose, param_intrinsic, CV_RGB(255, 255, 255));

    cvShowImage("Edge", img_result);
    cout << "position: " << modelPosition[0] << ", " << modelPosition[1] << ", " << modelPosition[2] << " angle: " << modelAngle[0] << ", " << modelAngle[1] << ", " << modelAngle[2] << endl;
    key = cvWaitKey(0);
    switch(key)
    {
    case 'w': // further 
      modelPosition[2] += 0.02f;
      break;
    case 's': // closer
      modelPosition[2] -= 0.02f;
      break;
    case 'a':
      modelAngle[2] += 5.f;
      break;
    case 'd':
      modelAngle[2] -= 5.f;
      break;
    case 'q':
      modelAngle[0] += 10.f;
      break;
    case 'e':
      modelAngle[0] -= 10.f;
      break;
    case 'z':
      modelAngle[1] += 30.f;
      break;
    case 'c':
      modelAngle[1] -= 30.f;
      break;
    case 'k':
      // save into image and xml
      char buf[50];

      // save edge template
      cvSetImageROI(img_result, bound);
      sprintf(buf, "/edge_template%03d.png", int_not);
      cvSaveImage((str_result_path + buf).c_str(), img_result);

      // fit lines
      IplImage* img = cvLoadImage((str_result_path + buf).c_str(), 0);
      lf.FitLine(img);
      sprintf(buf, "/%s_edge_template_line%03d.png", obj_name.c_str(), int_not);
      lf.DisplayEdgeMap(img, (str_result_path + buf).c_str());
      cvReleaseImage(&img);

      sprintf(buf, "/%s_edge_template%03d.txt", obj_name.c_str(), int_not);
      lf.SaveEdgeMap((str_result_path + buf).c_str());

      cvResetImageROI(img_result);

      // save template pose
      sprintf(buf, "/edge_template_pose%03d.xml", int_not);
      cvSave((str_result_path + buf).c_str(), pose);

      int_not++;
      break;
    }
    cvWaitKey(10);
  }

  cvDestroyWindow("Edge");
  cvReleaseMat(&pose);
  cvReleaseImage(&img_result);
  
  return (0);
}
