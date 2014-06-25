#include <iostream>
#include <string>

#include "tracker_irls.h"
#include "tracker_pf_texture.h"
#include "tracker_pf_textureless.h"

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>


namespace po = boost::program_options;

int main(int argc, char **argv)
{
  std::string tracker_name;
  std::string obj_name;
  std::string input;
  std::string intrinsic;
  std::string distortion;
  int width;
  int height;
  float sample_step;
  bool dull_edge;
  float th_cm;
  int n; // number of particles
  bool display;
  bool net;
  bool save_rslt_txt;
  bool save_rslt_img;
  std::string str_result_path;
  CvMat* pose_init;
  pose_init = cvCreateMat(4, 4, CV_32F);
  cvSetIdentity(pose_init);
  CV_MAT_ELEM(*pose_init, float, 2, 3) = 0.5f; // 0.5 meter in front of camera
  std::string pose_init_str;
  int min_keypoint_matches;
  std::string ach_channel;
  bool use_tracking;

  po::options_description desc("Tracker options");
  desc.add_options()
    ("help,h", "produce help message")
    ("tracker,t", po::value<std::string>(&tracker_name)->default_value("irls"), "name tracker (irls, pf, pf_textureless)")
    ("obj-name,o", po::value<std::string>(&obj_name), "name of traget object")
    ("input,i", po::value<std::string>(&input), "name of camera (e.g. flea) or image sequence path (e.g. dir/seq1/)")
    ("sample-step,s", po::value<float>(&sample_step)->default_value(0.005f), "sample step")
    ("num-particle,n", po::value<int>(&n)->default_value(1), "number of particles")
    ("save-txt", po::value<bool>(&save_rslt_txt)->default_value(false), "save results in text file")
    ("save-img", po::value<bool>(&save_rslt_img)->default_value(false), "save results in image files")
    ("save-path", po::value<std::string>(&str_result_path), "save results in image files")
    
    ("display", po::value<bool>(&display)->default_value(true), "display results or not")
    ("network", po::value<bool>(&net)->default_value(false), "use network mode or not")
    ("width", po::value<int>(&width)->default_value(640), "width")
    ("height", po::value<int>(&height)->default_value(480), "height")
    ("intrinsic", po::value<std::string>(&intrinsic)->default_value("Intrinsics_normal.xml"), "intrinsic parameters")
    ("distortion", po::value<std::string>(&distortion)->default_value("Distortion_normal.xml"), "distortion parameters")
    ("dull_edge", po::value<bool>(&dull_edge)->default_value(false), "consider dull edges")
    ("th_cm", po::value<float>(&th_cm)->default_value(0.2f), "threshold of chamfer matching")
    ("init_pose", po::value<std::string>(&pose_init_str), "init pose")
    // AKAN
    ("min_keypoint_matches", po::value<int>(&min_keypoint_matches)->default_value(20), "min number of keypoint matches to start tracking")
    ("use_ach_channel", po::value<std::string>(&ach_channel)->default_value("none"), "Use specific ach channel with given name")
    ("use_tracking", po::value<bool>(&use_tracking)->default_value(true), "Enable tracking after detection of object")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(argc < 2 || vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  if(input=="ach")
	{
		//sns_init();
	}

  if(obj_name.empty())
  {
    std::cerr << "obj-name should be specified." << std::endl;
    return 1;
  }

  if(save_rslt_txt || save_rslt_img)
  {
    if(str_result_path.empty())
    {
      std::cerr << "No save-path specified when using either save-txt or save-img" << std::endl;
      return 1;
    }
  }

  if(vm.count("input"))
  {
    std::cout << "input: " << vm["input"].as<std::string>() << std::endl;
  }

  if(vm.count("obj-name"))
  {
    std::cout << "obj-name: " << vm["obj-name"].as<std::string>() << std::endl;
  }

  if(vm.count("n"))
  {
    std::cout << "n: " << vm["n"].as<int>() << std::endl;
  }

  if(vm.count("init_pose"))
  {
    std::cout << "init_pose: ";
    std::vector<std::string> tokens;
    boost::split(tokens, pose_init_str, boost::is_any_of(","), boost::token_compress_on);
        
    for(int i = 0; i < tokens.size(); i++)
      std::cout << tokens[i] << " ";
    std::cout << std::endl;

    if(tokens.size() != 16)
      std::cerr << "Not enough number of data in pose. 16 values are required!" << std::endl;
    {
      // oeverwrite pose_init
      for (int i = 0; i < 16; i++)
        pose_init->data.fl[i] = std::atof(tokens[i].c_str());
    }
  }

  TrackerBase* tracker;
  if(tracker_name.compare("irls") == 0)
  {
    tracker = new IrlsTracker ();
    tracker->setMinKeypointMatches(min_keypoint_matches);
  }
  else if(tracker_name.compare("pf") == 0)
  {
    tracker = new TextureParticleFilterTracker ();
    ((TextureParticleFilterTracker*)tracker)->setNumParticle(n);
    ((TextureParticleFilterTracker*)tracker)->initParticleFilter();
  }
  else if(tracker_name.compare("pf_textureless") == 0)
  {
    tracker = new TexturelessParticleFilterTracker ();
    ((TexturelessParticleFilterTracker*)tracker)->setNumParticle(n);
    ((TexturelessParticleFilterTracker*)tracker)->setThresholdCM(th_cm);
    ((TexturelessParticleFilterTracker*)tracker)->initParticleFilter();
  }
  else
  {
    std::cerr << "Unknown tracker method name: " << tracker_name << std::endl;
    return 1;
  }

  tracker->setSampleStep(sample_step);
  tracker->setDisplay(display);
  tracker->setNetworkMode(net);
  tracker->setSaveResultText(save_rslt_txt);
  tracker->setSaveResultImage(save_rslt_img);
  tracker->setSaveResultPath(str_result_path);
  tracker->setConsideringDullEdges(dull_edge);
  tracker->setTracking(use_tracking);
    
  tracker->initTracker(obj_name, input, intrinsic, distortion, width, height, pose_init , ach_channel);
  tracker->run();
  delete tracker;

  return (0);
}
