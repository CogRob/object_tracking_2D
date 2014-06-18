/*
Copyright 2011, Ming-Yu Liu

All Rights Reserved 

Permission to use, copy, modify, and distribute this software and 
its documentation for any non-commercial purpose is hereby granted 
without fee, provided that the above copyright notice appear in 
all copies and that both that copyright notice and this permission 
notice appear in supporting documentation, and that the name of 
the author not be used in advertising or publicity pertaining to 
distribution of the software without specific, written prior 
permission. 

THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, 
INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
ANY PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR 
ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES 
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN 
AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING 
OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE. 
*/

//#include "stdafx.h"
//#include <cxcore.h>
//#include <cv.h>
//#include <highgui.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Image/Image.h"
#include "Image/ImageIO.h"
#include "Fitline/LFLineFitter.h"
#include "LMLineMatcher.h"

#include <iostream>
#include <string>

void DrawDetWind(IplImage *image,int x,int y,int detWindWidth,int detWindHeight,CvScalar scalar,int thickness)
{
    cvLine(image,cvPoint( x,y),cvPoint( x+detWindWidth,y),scalar,thickness);
    cvLine(image,cvPoint( x+detWindWidth,y),cvPoint( x+detWindWidth, y+detWindHeight),scalar,thickness);
    cvLine(image,cvPoint( x+detWindWidth,y+detWindHeight),cvPoint( x, y+detWindHeight),scalar,thickness);
    cvLine(image,cvPoint( x, y+detWindHeight),cvPoint( x, y),scalar,thickness);
}


int main(int argc, char *argv[])
{

    if(!(argc == 3 || argc == 4))
    {
        std::cerr<<"[Syntax] fdcm template.txt input_edgeMap.pgm input_realImage.jpg [OR]"<<std::endl;
        std::cerr<<"[Syntax] fdcm template.txt input_realImage.jpg"<<std::endl;
        exit(0);
    }

    LFLineFitter lf;
    LMLineMatcher lm;
    lf.Configure("para_line_fitter.txt");
    lm.Configure("para_line_matcher.txt");


    //Image *inputImage=NULL;
    IplImage *inputImage=NULL;
    //Image<uchar> *inputImage=NULL;
    IplImage *edgeImage = NULL;

    string templateFileName(argv[1]);
    string displayImageName(argc==4?argv[3]:argv[2]);

    //string templateFileName("Exp_Smoothness/template_list.txt");
    //string edgeMapName("Exp_Smoothness/device5-20_edge_cluttered.pgm");
    //string displayImageName("Exp_Smoothness/device5-20_edge_cluttered.pgm");	

    //string templateFileName("data/template_giraffe.txt");
    //string edgeMapName("data/looking_edges.pgm");
    //string displayImageName("data/looking.ppm");

    //string templateFileName("data/template_applelogo.txt");
    //string edgeMapName("data/hat_edges.pgm");
    //string displayImageName("data/hat.jpg");


    //inputImage = cvLoadImage(edgeMapName.c_str(),0);
    //inputImage = ImageIO::LoadPGM(edgeMapName.c_str());

    if(argc == 4)
    {
	std::cout << argc << std::endl;
        string edgeMapName(argv[2]);
        inputImage = cvLoadImage(edgeMapName.c_str(),0);
        if(inputImage==NULL)
        {
            std::cerr<<"[ERROR] Fail in reading image "<<edgeMapName<<std::endl;
            exit(0);
        }
    }
    else //if(argc == 3)
    {
        IplImage* img = cvLoadImage(displayImageName.c_str(),0);
        inputImage = cvCloneImage(img);
        //cvCanny(img, inputImage, 20, 40, 3);
        cvCanny(img, inputImage, 20, 80, 3);
        //cvCanny(img, inputImage, 80, 120, 3);
        cvReleaseImage(&img);

        edgeImage = cvCloneImage(inputImage);
    }

    lf.Init();
    lm.Init(templateFileName.c_str());


    // Line Fitting
    lf.FitLine(inputImage);

    // FDCM Matching
    vector<LMDetWind> detWind;
    //lm.Match(lf, detWind);

    vector<vector<LMDetWind> > detWinds(lm.ndbImages_);
    vector<LMDetWind> detWindAll;
    double maxThreshold = 0.30;
    for(int i=0; i<lm.ndbImages_; i++)
    {
        std::cout << "[" << i << "]-th template ..." << std::endl;
        lm.SingleShapeDetectionWithVaryingQuerySize(lf, i, maxThreshold, detWinds[i]);
        for(size_t j=0; j<detWinds[i].size(); j++)
        {
            detWindAll.push_back(detWinds[i][j]);
        }
    }

    // Sort the window array in the ascending order of matching cost.
    LMDetWind *tmpWind = new LMDetWind[detWindAll.size()];
    for(size_t i=0;i<detWindAll.size();i++)
        tmpWind[i] = detWindAll[i];
    MMFunctions::Sort(tmpWind, detWindAll.size());
    for(size_t i=0;i<detWindAll.size();i++)
        detWind.push_back(tmpWind[i]);
    delete [] tmpWind;

    //MMFunctions::Sort(detWindAll,detWindAll.size());
    //detWind = detWindAll;
    
    //int last = detWind2.size()-1;
    //int nDetWindows = detWind2[last].size();
    //detWind = detWind2[last];



    //lm.MatchCostMap(lf,outputCostMapName.c_str());

    // Display best matcher in edge map
    //if(displayImageName.c_str())
    //{
    //	Image<RGBMap> *debugImage = ImageIO::LoadPPM(displayImageName.c_str());
    //	LMDisplay::DrawDetWind(debugImage,detWind[0].x_,detWind[0].y_,detWind[0].width_,detWind[0].height_,RGBMap(0,255,0),4);
    //	char outputname[256];
    //	sprintf(outputname,"%s.output.ppm",displayImageName.c_str());
    //	ImageIO::SavePPM(debugImage,outputname);
    //	delete debugImage;
    //}

    std::cout << detWind.size() << " detections..." << std::endl;

    
    IplImage* dispImage = cvLoadImage(displayImageName.c_str());

    for(size_t i=0; i<detWind.size(); i++)
    {
        std::cout << detWind[i].x_ << " " << detWind[i].y_ << " " << detWind[i].width_ << " " << detWind[i].height_ << " " << detWind[i].cost_ << " " << detWind[i].count_ << " " << detWind[i].scale_ << " " << detWind[i].aspect_ << " " << detWind[i].tidx_ << std::endl;
    }

    for(size_t i=1; i<(detWind.size()<10?detWind.size():10); i++)
        DrawDetWind(dispImage, detWind[i].x_, detWind[i].y_, detWind[i].width_, detWind[i].height_, cvScalar(255,255,0), i==1?2:1);

    if(detWind.size() > 0)
        DrawDetWind(dispImage, detWind[0].x_, detWind[0].y_, detWind[0].width_, detWind[0].height_, cvScalar(0,255,255), 2);

    cvNamedWindow("edge", 1);
    cvNamedWindow("output", 1);
    cvShowImage("edge", edgeImage);
    cvShowImage("output", dispImage);
    cvSaveImage("result.png", dispImage);
    //cvWaitKey(0);

    cvDestroyWindow("edge");
    cvDestroyWindow("output");

    if(inputImage)  cvReleaseImage(&inputImage);
    if(dispImage)   cvReleaseImage(&dispImage);
    if(edgeImage)   cvReleaseImage(&edgeImage);
    
    return 0;
};
