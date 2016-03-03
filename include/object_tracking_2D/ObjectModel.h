#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <math.h>
#include <GL/glew.h>
#include <GL/gl.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "glm.h" // GLM: model interface of OJB (Wavefront Technologies) file

using namespace std;

class CEdgeTracker;
class CPoseEstimationSURF;

class CObjectModel
{
public:
  struct SamplePoint
  {
    CvPoint3D32f coord3;        // Coordinate of a visible 3D sampling point
    CvPoint2D32f coord2;        // Coordinate of a visible 2D sampling point
    CvPoint2D32f nuv;           // Normal unit vector of a visible 2D sampling point
    unsigned char nidx;         // Normal index of a visible 2D sampling point (0, 1, 2, 3)
    double dist;                // Normal distance of a visible 3D sampling point
    double normal_ang;          // Normal angle (4 ways)
    double normal_ang_deg;      // Normal angle in degree (accurate)
    double dx;                  // Normal unit vector (x)
    double dy;                  // Normal unit vector (y)
    CvPoint2D32f edge_pt2;      // corresponding edge coordinate
    int edge_mem;               // Edge membership (0, 1, 2, ... until # of edges -1)
  };

  CObjectModel(string obj_name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker);
  ~CObjectModel(void);

  void loadObjectCADModel(const std::string& obj_name);
  void findVisibleSamplePoints(void);

  void setProjectionMatrix(CvMat* intrinsic);
  void setModelviewMatrix(CvMat* pose);
  void findNormalUsingEdgeCoord(void);
  void findNormalUsingEdgeCoordCoarseOri(void);
  void findNormalUsingEdgeCoordFineOri(void);    
  CvPoint2D32f project3Dto2D(CvPoint3D32f &pt3);
  CvPoint2D32f project3Dto2D(CvPoint3D32f &pt3, CvMat* pose);

  void saveKeyframe(string obj_name, IplImage* imgG, CvMat* pose);
  void loadKeyframes(string obj_name);
  void loadEdgeTemplates(const std::string& obj_name);

  inline std::vector<IplImage*>& getKeyframeImages()    { return keyframe_images_;              }
  inline std::vector<CvMat*>& getKeyframePoses()        { return keyframe_poses_;               }
  inline std::vector<CvMat*>& getKeyframeKeypoints2D()  { return keyframe_keypoints_2d_;        }
  inline std::vector<CvMat*>& getKeyframeKeypoints3D()  { return keyframe_keypoints_3d_;        }
  inline std::vector<CvMat*>& getKeyframeDescriptors()  { return keyframe_descriptors_;         }
  int getNumOfKeyframes()                               { return num_keyframes_;                }
  int getNumOfEdgeTemplates()                           { return num_edge_templates_;           }
  inline CvMat* getEdgeTemplatePose(int i)              { return edge_template_poses_[i];       }

  inline vector<SamplePoint>& getVisibleSamplePoints()  { return visible_sample_points_;        }
  inline int getNumberOfVisibleSamplePoints()           { return visible_sample_points_.size(); }
  bool isEnoughValidSamplePoints(double th_ratio);
  double GetValidVisibleSamplePointsRatio();

  void findEdgeCorrespondences();
  void findEdgeCorrespondencesCoarseOri();
  void findEdgeCorrespondencesFineOri();
  int refineEdgeCorrespondences_RANSAC(CvMat *E, int N=1000, double th=10.0);
  void extractEdge(IplImage* img, int smoothSize=1, int cannyLow=20, int cannyHigh=40, IplImage* edge = NULL);
  void extractEdges(IplImage* img, int smoothSize=1, int cannyLow=20, int cannyHigh=40, IplImage* edge = NULL, int image_num =0, string path = "");
  void extractEdgeOri(IplImage* img, int smoothSize=1);
  void drawPointsAndErrorCoarseOri(IplImage* img_dest);
  void drawPointsAndErrorFineOri(IplImage* img_dest);
  float getTriangleArea(GLMmodel* m, int tri_idx);
  // display functions
  void displayPoseLine(IplImage* img_dest, CvMat* pose, CvScalar color, int thickness, bool onright);
  void displaySamplePointsAndErrors(IplImage* img_dest);

protected:
  bool initOpenGL(int width, int height);
  bool initFrameBufferObject(int width, int height);
  void _saveSURFkeypoints(std::string& obj_name, IplImage* imgG, CvMat* pose, vector<CvPoint2D32f>& new2Dpt, vector<vector<float> >& newDesc, vector<CvPoint3D32f>& new3Dpt);
  void generateSamplePoints(std::vector<CvPoint3D32f>& sample_points, std::vector<int>& edge_index, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& edges);
  void determineSharpEdges(GLMmodel* model, float th_sharp, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& sharp_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_normals);
  void determineSharpEdges_slow(GLMmodel* model, float th_sharp, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& sharp_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_edges, std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> >& dull_normals);

  void Cross(float* u, float* v, float* n)
  {
    assert(u); assert(v); assert(n);

    n[0] = u[1]*v[2] - u[2]*v[1];
    n[1] = u[2]*v[0] - u[0]*v[2];
    n[2] = u[0]*v[1] - u[1]*v[0];
  }

  inline CvPoint3D32f getPoint(CvMat* mat, int row)
  {
    return cvPoint3D32f(CV_MAT_ELEM(*mat, float, row, 0), CV_MAT_ELEM(*mat, float, row, 1), CV_MAT_ELEM(*mat, float, row, 2));
  }

  inline void setPoint(CvMat* mat, int row, CvPoint3D32f& pt)
  {
    CV_MAT_ELEM(*mat, float, row, 0) = pt.x;
    CV_MAT_ELEM(*mat, float, row, 1) = pt.y;
    CV_MAT_ELEM(*mat, float, row, 2) = pt.z;
  }

  bool _withinOri(float o1, float o2, float oth);

  inline int round(double d) { return (int)ceil(d - 0.5); }

  int width_;
  int height_;
  // sampled points on sharp edges
  std::vector<CvPoint3D32f> sharp_sample_points_;
  // edge indices of each sharp sample point
  std::vector<int> sharp_sample_points_edge_indices_;
  // sample points on dull edges
  // note that dull edges are used for determining boundary edges
  // that's why dull sample points are saved for each edge
  std::vector<std::vector<CvPoint3D32f> > set_dull_sample_points_;
  std::vector<int> dull_boundary_sample_points_edge_indices_;
  // every visible sample points on edges (sharp and dull)
  std::vector<SamplePoint> visible_sample_points_;

  IplImage* img_edge_;
  IplImage* img_gx_;
  IplImage* img_gy_;

  CvMat* pose_;
  CvMat* intrinsic_;
  GLMmodel* meshmodel_;

  std::vector<IplImage*>  keyframe_images_;
  std::vector<CvMat*>     keyframe_poses_;
  std::vector<CvMat*>     keyframe_keypoints_2d_;
  std::vector<CvMat*>     keyframe_keypoints_3d_;
  std::vector<CvMat*>     keyframe_descriptors_;
  int                     num_keyframes_;

  std::vector<CvMat*>     edge_template_poses_;
  int                     num_edge_templates_;

  GLuint dl_;

  float sample_step_;
  int maxd_;
  bool dulledge_;
  int image_number;


  CEdgeTracker* edge_tracker_;
  bool use_fine_orientation_;
  int draw_type_;
  bool include_starting_point_;

  GLuint framebuffer_;
  GLuint renderbuffer_;
  GLuint depthbuffer_;

  float th_sharp_; // 0.0 ~ 1.0 (0.9)

  std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> > sharp_edges_;
  std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> > dull_edges_;
  std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> > dull_normals_;
  std::vector<std::pair<CvPoint3D32f, CvPoint3D32f> > dull_boundary_edges_;
};
