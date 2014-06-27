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

#include "LFLineFitter.h"
#include "../include/object_tracking_2D/Timer.h"
#include <omp.h> // openmp

LFLineFitter::LFLineFitter()
{
	localWindSize_ = 50;
	smallLocalWindowSize_ = max(localWindSize_/10,5);
	nMinEdges_ = 5;
	nMaxWindPoints_ = 4*(localWindSize_+1)*(localWindSize_+1);
	minLength_ = 2;

	nLinesToFitInStage_[0] = 300;
	nLinesToFitInStage_[1] = 3000;
	nTrialsPerLineInStage_[0] = 300;
	nTrialsPerLineInStage_[1] = 1;
	sigmaFitALine_ = 0.75;
	sigmaFindSupport_ = 0.75;
	maxGap_ = 1.5;

	outEdgeMap_ = NULL;
	rpoints_ = NULL;
	rProjection_ = NULL;
	absRProjection_ = NULL;
	idx_ = NULL;
}

void LFLineFitter::Configure(double sigmaFitALine,double sigmaFindSupport, double maxGap,int nLayer,int *nLinesToFitInStage,int *nTrialsPerLineInStage)
{
	sigmaFitALine_ = sigmaFitALine;
	sigmaFindSupport_ = sigmaFindSupport;
	maxGap_ = maxGap;
	for(int i=0;i<nLayer;i++)
	{
		nLinesToFitInStage_[i] = nLinesToFitInStage[i];
		nTrialsPerLineInStage_[i] = nTrialsPerLineInStage[i];
	}
}


LFLineFitter::~LFLineFitter()
{
	SafeRelease();
}

void LFLineFitter::SafeRelease()
{
	if(outEdgeMap_)
		delete [] outEdgeMap_;
	if(rpoints_)
		delete [] rpoints_;
	if(rProjection_)
		delete [] rProjection_;
	if(absRProjection_)
		delete [] absRProjection_;
	if(idx_)
		delete [] idx_;
	outEdgeMap_ = NULL;
	rpoints_ = NULL;
	rProjection_ = NULL;
	absRProjection_ = NULL;
	idx_ = NULL;


}

void LFLineFitter::Init()
{
	outEdgeMap_ = new LFLineSegment[nLinesToFitInStage_[0]+nLinesToFitInStage_[1]];
	//rpoints_ = new Point<int> [nMaxWindPoints_];
    rpoints_ = new CvPoint [nMaxWindPoints_];
	rProjection_ = new double [nMaxWindPoints_];
	absRProjection_ = new double [nMaxWindPoints_];
	idx_ = new int [nMaxWindPoints_];
	
}

//void LFLineFitter::FitLine(Image<unsigned char> *inputImage)
//{
//	//LARGE_INTEGER t1, t2, f;
//	//QueryPerformanceFrequency(&f);
//	//QueryPerformanceCounter(&t1);
//
//	width_ = inputImage->width();
//	height_ = inputImage->height();
//
//	map<int,Point<int> > edgeMap;
//
//	int i,j,k;
//	int x0,y0;
//	int width,height;
//	int index=0;
//	int nPixels=0;
//	int nEdges=0;
//	int maxSupport=0;	
//	LFLineSegment tmpLs,bestLs;
//	Point<double> lnormal;
//	int nWindPoints=0,nWaitingKillingList=0,nProposedKillingList=0;
//	Point<int> *windPoints,*waitingKillingList,*proposedKillingList;
//	windPoints = new Point<int> [nMaxWindPoints_];
//	waitingKillingList = new Point<int>[nMaxWindPoints_];
//	proposedKillingList = new Point<int>[nMaxWindPoints_];
//
//	width = inputImage->width();
//	height = inputImage->height();
//	nPixels = width*height;
//
//	for(int y=0;y<height;y++)
//	{
//		for(int x=0;x<width;x++)
//		{
//			i = x + y*width;
//			//if(cvGetReal1D(inputImage,i)!=0)
//			if(imRef(inputImage,x,y)!=0)
//			{				
//				edgeMap.insert(pair<int,Point<int> >(i,Point<int>(x,y)));
//				nEdges++;
//			}
//		}
//	}
//
//
//	nInputEdges_ = nEdges;
//	nLineSegments_=0;
//	for(k=0;k<2;k++)
//	{
//		if(nEdges<nMinEdges_)
//			break;
//		for(i=0;i<nLinesToFitInStage_[k];i++)
//		{
//			maxSupport = 0;
//			for(j=0;j<nTrialsPerLineInStage_[k];j++)
//			{
//				// Sample a point
//				index = SampleAPixel(&edgeMap,inputImage,nPixels);
//				y0 = index/width;
//				x0 = index - y0*width;
//				
//				// Locate the subwindow
//				Find(x0,y0,windPoints,nWindPoints,inputImage,smallLocalWindowSize_);				
//				
//				// Infer a line direction
//				FitALine(nWindPoints,windPoints,sigmaFitALine_,lnormal);
//
//				// Locate the subwindow			
//				Find(&edgeMap,x0,y0,windPoints,nWindPoints,inputImage,localWindSize_);
//
//				// Find the support			
//				FindSupport(nWindPoints,windPoints,lnormal,sigmaFindSupport_,maxGap_,tmpLs,proposedKillingList,nProposedKillingList,x0,y0);
//				
//				// Check if need to update
//				if(tmpLs.nSupport_ > maxSupport)
//				{
//					maxSupport = tmpLs.nSupport_;
//					nWaitingKillingList = nProposedKillingList;
//					memcpy(waitingKillingList,proposedKillingList,sizeof(Point<int>)*nWaitingKillingList);
//					bestLs = tmpLs;
//				}
//			}
//
//			// Remove points
//			for(j=0;j<maxSupport;j++)
//			{
//				//cvSetReal2D(inputImage,waitingKillingList[j].y,waitingKillingList[j].x,0.0);
//				imRef(inputImage,waitingKillingList[j].x,waitingKillingList[j].y) = 0;
//				edgeMap.erase(waitingKillingList[j].y*width+waitingKillingList[j].x);
//			}
//			nEdges -= bestLs.nSupport_;
//			bestLs.len_ = sqrt( (bestLs.sx_-bestLs.ex_)*(bestLs.sx_-bestLs.ex_) + (bestLs.sy_-bestLs.ey_)*(bestLs.sy_-bestLs.ey_));
//			outEdgeMap_[nLineSegments_] = bestLs;
//			nLineSegments_++;
//
//			if(nEdges<nMinEdges_)
//				break;
//		}
//	}
//	MMFunctions::Sort(outEdgeMap_,nLineSegments_,0);
//	delete [] windPoints;
//	delete [] waitingKillingList;
//	delete [] proposedKillingList;
//	edgeMap.clear();
//	////////QueryPerformanceCounter(&t2);
//	//cout<<"[DO] Fit "<<nLineSegments_<<" lines taking "<<setiosflags(ios::fixed)<<setprecision(6)<<(t2.QuadPart - t1.QuadPart)/(1.0*f.QuadPart)<<"seconds"<<endl;
//}

void LFLineFitter::FitLine(IplImage *inputImage)
{
    Timer timer;
    timer.start();

    width_ = inputImage->width;
    height_ = inputImage->height;

    map<int,CvPoint> edgeMap;

    int i,j,k;
    int x0,y0;
    int width,height;
    int index=0;
    int nPixels=0;
    int nEdges=0;
    int maxSupport=0;	
    LFLineSegment tmpLs,bestLs;
    CvPoint2D64f lnormal;
    int nWindPoints=0,nWaitingKillingList=0,nProposedKillingList=0;
    CvPoint *windPoints,*waitingKillingList,*proposedKillingList;
    windPoints = new CvPoint [nMaxWindPoints_];
    waitingKillingList = new CvPoint[nMaxWindPoints_];
    proposedKillingList = new CvPoint[nMaxWindPoints_];

    width = inputImage->width;
    height = inputImage->height;
    nPixels = width*height;

    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            i = x + y*width;
            if(cvGetReal1D(inputImage,i)!=0)
            {
                edgeMap.insert(pair<int,CvPoint>(i,cvPoint(x,y)));
                nEdges++;
            }
        }
    }

    nInputEdges_ = nEdges;
    nLineSegments_=0;
    for(k=0;k<2;k++)
    {
        if(nEdges<nMinEdges_)
            break;
        for(i=0;i<nLinesToFitInStage_[k];i++)
        {
            maxSupport = 0;
            for(j=0;j<nTrialsPerLineInStage_[k];j++)
            {
                // Sample a point
                index = SampleAPixel(&edgeMap,inputImage,nPixels);
                y0 = index/width;
                x0 = index - y0*width;

                // Locate the subwindow
                Find(x0,y0,windPoints,nWindPoints,inputImage,smallLocalWindowSize_);				

                // Infer a line direction
                FitALine(nWindPoints,windPoints,sigmaFitALine_,lnormal);

                // Locate the subwindow			
                Find(&edgeMap,x0,y0,windPoints,nWindPoints,inputImage,localWindSize_);

                // Find the support			
                FindSupport(nWindPoints,windPoints,lnormal,sigmaFindSupport_,maxGap_,
                    tmpLs,proposedKillingList,nProposedKillingList,x0,y0);

                // Check if need to update
                if(tmpLs.nSupport_ > maxSupport)
                {
                    maxSupport = tmpLs.nSupport_;
                    nWaitingKillingList = nProposedKillingList;
                    memcpy(waitingKillingList,proposedKillingList,sizeof(CvPoint)*nWaitingKillingList);
                    bestLs = tmpLs;
                }
            }

            // Remove points
            for(j=0;j<maxSupport;j++)
            {
                cvSetReal2D(inputImage,waitingKillingList[j].y,waitingKillingList[j].x,0.0);

                edgeMap.erase(waitingKillingList[j].y*width+waitingKillingList[j].x);
            }
            nEdges -= bestLs.nSupport_;
            bestLs.len_ = sqrt( (bestLs.sx_-bestLs.ex_)*(bestLs.sx_-bestLs.ex_) + (bestLs.sy_-bestLs.ey_)*(bestLs.sy_-bestLs.ey_));
            outEdgeMap_[nLineSegments_] = bestLs;
            nLineSegments_++;

            if(nEdges<nMinEdges_)
                break;
        }
    }

    qsort(outEdgeMap_,nLineSegments_,sizeof(LFLineSegment),LFLineSegment::Compare);
    std::cout<<"Num. lines in test image = "<<nLineSegments_<<std::endl;

    delete [] windPoints;
    delete [] waitingKillingList;
    delete [] proposedKillingList;
    edgeMap.clear();

    timer.printTimeMilliSec("FitLine()");
}

void LFLineFitter::FitLine_omp(IplImage *inputImage)
{
    Timer timer;
    timer.start();

    width_ = inputImage->width;
    height_ = inputImage->height;

    map<int, CvPoint> edgeMap;

    int width,height;
    int nPixels=0;
    int nEdges=0;

    width = inputImage->width;
    height = inputImage->height;
    nPixels = width*height;

    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            int i = x + y*width;
            if(cvGetReal1D(inputImage,i)!=0)
            {
                edgeMap.insert(pair<int,CvPoint>(i,cvPoint(x,y)));
                nEdges++;
            }
        }
    }

    nInputEdges_ = nEdges;
    nLineSegments_ = 0;

    for(int k=0; k<2; k++)
    {
        if(nEdges < nMinEdges_)
            break;

        for(int i=0;i<nLinesToFitInStage_[k];i++)
        {
            int maxSupport = 0;
            LFLineSegment bestLs;
            CvPoint *waitingKillingList;
            waitingKillingList = new CvPoint[nMaxWindPoints_];

#pragma omp parallel for
            for(int j=0; j<nTrialsPerLineInStage_[k]; j++)
            {
                // Sample a point
                int index = SampleAPixel(&edgeMap, inputImage, nPixels);
                int y0 = index/width;
                int x0 = index - y0*width;

                CvPoint *windPoints;
                windPoints = new CvPoint [nMaxWindPoints_];
                CvPoint *proposedKillingList;
                proposedKillingList = new CvPoint[nMaxWindPoints_];

                // Locate the subwindow
                int nWindPoints;
                Find(x0, y0, windPoints, nWindPoints, inputImage, smallLocalWindowSize_);

                // Infer a line direction
                CvPoint2D64f lnormal;
                FitALine(nWindPoints, windPoints, sigmaFitALine_, lnormal);
                
                // Locate the subwindow			
                Find(&edgeMap, x0, y0, windPoints, nWindPoints, inputImage, localWindSize_);

                // Find the support
                LFLineSegment tmpLs;
                int nProposedKillingList;
                FindSupport(nWindPoints, windPoints, lnormal, sigmaFindSupport_, maxGap_,
                    tmpLs, proposedKillingList, nProposedKillingList, x0, y0);

#pragma omp critical
                {
                    // Check if need to update
                    if(tmpLs.nSupport_ > maxSupport)
                    {
                        maxSupport = tmpLs.nSupport_;
                        int nWaitingKillingList = nProposedKillingList;
                        memcpy(waitingKillingList, proposedKillingList, sizeof(CvPoint)*nWaitingKillingList);
                        bestLs = tmpLs;
                    }
                }

                delete [] windPoints;
                delete [] proposedKillingList;
            }

            // Remove points
            for(int j=0;j<maxSupport;j++)
            {
                cvSetReal2D(inputImage, waitingKillingList[j].y, waitingKillingList[j].x, 0.0);
                edgeMap.erase(waitingKillingList[j].y * width + waitingKillingList[j].x);
            }
            nEdges -= bestLs.nSupport_;
            bestLs.len_ = sqrt( (bestLs.sx_-bestLs.ex_)*(bestLs.sx_-bestLs.ex_) + (bestLs.sy_-bestLs.ey_)*(bestLs.sy_-bestLs.ey_));
            outEdgeMap_[nLineSegments_] = bestLs;
            nLineSegments_++;
            
            delete [] waitingKillingList;

            if(nEdges < nMinEdges_)
                break;
        }
    }

    qsort(outEdgeMap_,nLineSegments_,sizeof(LFLineSegment),LFLineSegment::Compare);
    std::cout<<"Num. lines in test image = "<<nLineSegments_<<std::endl;

    edgeMap.clear();

    timer.printTimeMilliSec("FitLine()");
}

void LFLineFitter::SaveEdgeMap(const char *filename)
{
	FILE *fp;
	MMFunctions::Sort(outEdgeMap_,nLineSegments_,0);
	fp = fopen(filename,"wt");

	fprintf(fp,"%d %d\n",width_,height_);
	fprintf(fp,"%d\n",nLineSegments_);

	double ratio=0;
	double count=0;
	for(int i=0;i<nLineSegments_;i++)
	{
		count += (double)outEdgeMap_[i].nSupport_;
		ratio = count/nInputEdges_;
		fprintf(fp,"%d %d %d %d\n",(int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_,(int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_);
	}
	fclose(fp);
}

void LFLineFitter::LoadEdgeMap(const char *filename)
{
	SafeRelease();
	FILE *fp = NULL;
	fp = fopen(filename,"rt");
	if(fp==NULL)
	{
		cerr<<"Cannot read "<<filename<<endl;
		exit(-1);
	}

	fscanf(fp,"%d %d\n",&width_,&height_);
	fscanf(fp,"%d\n",&nLineSegments_);

	outEdgeMap_ = new LFLineSegment [nLineSegments_];
	for(int i=0;i<nLineSegments_;i++)
	{
		fscanf(fp,"%lf %lf %lf %lf\n",&outEdgeMap_[i].sx_,&outEdgeMap_[i].sy_,&outEdgeMap_[i].ex_,&outEdgeMap_[i].ey_);
	}
	fclose(fp);
}

//Image<uchar> *LFLineFitter::ComputeOuputLineImage(Image<uchar> *inputImage)
//{
//
//	Image<uchar> *debugImage = new Image<uchar>(inputImage->width(),inputImage->height());
//	for(int i=0;i<nLineSegments_;i++)
//	{
//		ImageDraw<uchar>::Line(debugImage,(int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_,(int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_,255);
//	}
//	return debugImage;
//}

//void LFLineFitter::DisplayEdgeMap(Image<uchar> *inputImage,const char *outputImageName)
//{
//	// for debug
//	//Image *debugImage = cvCreateImage( cvSize(inputImage->width,inputImage->height), IPL_DEPTH_8U,1);
//	//cvZero(debugImage);
//
//	Image<uchar> debugImage(inputImage->width(),inputImage->height());;
//
//	for(int i=0;i<nLineSegments_;i++)
//	{
//		//cvLine(debugImage,cvPoint((int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_),cvPoint((int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_),cvScalar(255));
//		ImageDraw<uchar>::Line(&debugImage,(int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_,(int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_,255);
//	}
//
//	if(outputImageName!=NULL)
//	{
//		printf("Save Image %s\n\n",outputImageName);
//		//cvSaveImage(outputImageName,debugImage);
//		ImageIO::SavePGM(&debugImage,outputImageName);
//	}
//	//else
//	//{		
//	//	cvNamedWindow("debug",1);
//	//	cvShowImage("debug",debugImage);
//	//	cvWaitKey(0);
//	//}
//	//cvReleaseImage(&debugImage);
//}

void LFLineFitter::DisplayEdgeMap(IplImage *inputImage,const char *outputImageName)
{
	// debug
	IplImage *debugImage = cvCreateImage( cvSize(inputImage->width,inputImage->height), IPL_DEPTH_8U,1);
	cvZero(debugImage);

	for(int i=0;i<nLineSegments_;i++)
	{
		cvLine(debugImage,cvPoint((int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_),
			cvPoint((int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_),cvScalar(255));
	}

	if(outputImageName!=NULL)
	{
		printf("Save Image %s\n\n",outputImageName);
		cvSaveImage(outputImageName,debugImage);
	}
	else
	{		
		cvNamedWindow("debug",1);
		cvShowImage("debug",debugImage);
		cvWaitKey(0);
	}

	cvReleaseImage(&debugImage);
}

void LFLineFitter::GetEdgeMap(IplImage *inputImage)
{
    assert(inputImage->nChannels == 1); // it should be gray image
    cvZero(inputImage);

    for(int i=0;i<nLineSegments_;i++)
    {
        cvLine(inputImage,cvPoint((int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_),
            cvPoint((int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_),cvScalar(255));
    }
}

void LFLineFitter::GetEdgeOriMap(IplImage *gx, IplImage *gy)
{
    assert(gx->nChannels == 1 && gy->nChannels == 1); // it should be gray image
    assert(gx->depth == 32 && gy->depth == 32); // it should be float (32)
    //cvSet(intputImage, cvScalar(-2.0f*M_PI)); // init to -2pi
    cvZero(gx);
    cvZero(gy);

    for(int i=0;i<nLineSegments_;i++)
    {
        //// skip a pixel point
        //if(abs((int)outEdgeMap_[i].sx_ - (int)outEdgeMap_[i].ex_) == 0 && abs((int)outEdgeMap_[i].sy_ - (int)outEdgeMap_[i].ey_) == 0)
        //    continue;
        //float ori = atan2(outEdgeMap_[i].ey_ - outEdgeMap_[i].sy, outEdgeMap_[i].ex_ - outEdgeMap_[i].sx); 
        // rotate 90 degree (convert from line ori to normal ori) x' = -y, y' = x
        double dx = -(outEdgeMap_[i].ey_ - outEdgeMap_[i].sy_);
        double dy = outEdgeMap_[i].ex_ - outEdgeMap_[i].sx_;
        
        cvLine(gx,cvPoint((int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_),
            cvPoint((int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_), cvScalar(dx));
        cvLine(gy,cvPoint((int)outEdgeMap_[i].sx_,(int)outEdgeMap_[i].sy_),
            cvPoint((int)outEdgeMap_[i].ex_,(int)outEdgeMap_[i].ey_), cvScalar(dy));
    }
}


//void LFLineFitter::FindSupport(const int nWindPoints,Point<int> *windPoints,Point<double> &lnormal,
//							 double sigmaFindSupport,double maxGap,LFLineSegment &ls,
//							 Point<int> *proposedKillingList,int &nProposedKillingList,int x0,int y0)
//{
//	int i,j;	
//	int nRindices = 0;
//	int zeroIndex=0;
//	double residuals;
//	Point<double> ldirection;
//	
//	// Find the point within the threshold by taking dot product
//	for(i=0;i<nWindPoints;i++)
//	{
//		residuals = abs( windPoints[i].x*lnormal.x + windPoints[i].y*lnormal.y );
//		if(residuals<sigmaFindSupport)
//		{
//			rpoints_[nRindices] = windPoints[i];
//			nRindices++;
//		}
//	}
//	ldirection.x = -lnormal.y;
//	ldirection.y = lnormal.x;
//
//
//	if(nRindices<minLength_)
//	{
//		ls.nSupport_ = -1;
//		return;
//	}
//
//
//	// Project to the line
//	for(i=0;i<nRindices;i++)
//	{
//		rProjection_[i] = rpoints_[i].x*ldirection.x+ rpoints_[i].y*ldirection.y;		
//		idx_[i] = i;
//	}
//	
//	// Sort the projection and find the starting and ending points	
//	MMFunctions::ISort(rProjection_,nRindices,idx_);
//
//	for(i=0;i<nRindices;i++)
//		absRProjection_[i] = abs(rProjection_[i]);
//
//	for(i=0;i<nRindices;i++)
//	{
//		if(absRProjection_[i]==0)
//		{
//			zeroIndex = i;
//			break;
//		}
//	}
//	
//	int maxIndex = nRindices-1;
//	for( i=zeroIndex; i<(nRindices-1); i++)
//	{	
//		if((rProjection_[i+1]-rProjection_[i])>maxGap)
//		{	
//			maxIndex = i;
//			break;
//		}
//	}
//
//	int minIndex = 0;
//	for( i=zeroIndex; i>0; i--)
//	{	
//		if((rProjection_[i]-rProjection_[i-1])>maxGap)
//		{	
//			minIndex = i;
//			break;
//		}
//	}
//
//	ls.nSupport_ = maxIndex-minIndex+1;
//	ls.sx_ = (double)rpoints_[ idx_[minIndex] ].x+x0;
//	ls.sy_ = (double)rpoints_[ idx_[minIndex] ].y+y0;
//	ls.ex_ = (double)rpoints_[ idx_[maxIndex] ].x+x0;
//	ls.ey_ = (double)rpoints_[ idx_[maxIndex] ].y+y0;
//	
//	j = 0;
//	for(i=minIndex;i<=maxIndex;i++)
//	{
//		proposedKillingList[j].x = rpoints_[ idx_[i] ].x + x0;
//		proposedKillingList[j].y = rpoints_[ idx_[i] ].y + y0;
//		j++;
//	}
//	nProposedKillingList = j;
//
//	ls.normal_ = lnormal;
//}

void LFLineFitter::FindSupport(const int nWindPoints,CvPoint *windPoints,CvPoint2D64f &lnormal,
							 double sigmaFindSupport,double maxGap,LFLineSegment &ls,
							 CvPoint *proposedKillingList,int &nProposedKillingList,int x0,int y0)
{
#if 1
	int i,j;	
	int nRindices = 0;
	int zeroIndex=0;
	double residuals;
	CvPoint2D64f ldirection;
	
	// Find the point within the threshold by taking dot product
	for(i=0;i<nWindPoints;i++)
	{
		residuals = abs( windPoints[i].x*lnormal.x + windPoints[i].y*lnormal.y );
		if(residuals<sigmaFindSupport)
		{
			rpoints_[nRindices] = windPoints[i];
			nRindices++;
		}
	}
	ldirection.x = -lnormal.y;
	ldirection.y = lnormal.x;


	if(nRindices<minLength_)
	{
		ls.nSupport_ = -1;
		return;
	}


	// Project to the line
	for(i=0;i<nRindices;i++)
	{
		rProjection_[i] = rpoints_[i].x*ldirection.x+ rpoints_[i].y*ldirection.y;		
		idx_[i] = i;
	}
	
	// Sort the projection and find the starting and ending points	
	ISort(rProjection_,nRindices,idx_);

	for(i=0;i<nRindices;i++)
		absRProjection_[i] = abs(rProjection_[i]);

	for(i=0;i<nRindices;i++)
	{
		if(absRProjection_[i]==0)
		{
			zeroIndex = i;
			break;
		}
	}
	
	int maxIndex = nRindices-1;
	for( i=zeroIndex; i<(nRindices-1); i++)
	{	
		if((rProjection_[i+1]-rProjection_[i])>maxGap)
		{	
			maxIndex = i;
			break;
		}
	}

	int minIndex = 0;
	for( i=zeroIndex; i>0; i--)
	{	
		if((rProjection_[i]-rProjection_[i-1])>maxGap)
		{	
			minIndex = i;
			break;
		}
	}

	ls.nSupport_ = maxIndex-minIndex+1;
	ls.sx_ = (double)rpoints_[ idx_[minIndex] ].x+x0;
	ls.sy_ = (double)rpoints_[ idx_[minIndex] ].y+y0;
	ls.ex_ = (double)rpoints_[ idx_[maxIndex] ].x+x0;
	ls.ey_ = (double)rpoints_[ idx_[maxIndex] ].y+y0;
	
	j = 0;
	for(i=minIndex;i<=maxIndex;i++)
	{
		proposedKillingList[j].x = rpoints_[ idx_[i] ].x + x0;
		proposedKillingList[j].y = rpoints_[ idx_[i] ].y + y0;
		j++;
	}
	nProposedKillingList = j;

	ls.normal_ = lnormal;
#else
    int i, j;	
	int nRindices = 0;
	int zeroIndex=0;
	double residuals;
	CvPoint2D64f ldirection;
	
    CvPoint *rpoints = new CvPoint [nMaxWindPoints_];
    double* rProjection = new double [nMaxWindPoints_];
	double *absRProjection = new double [nMaxWindPoints_];
	int *idx = new int [nMaxWindPoints_];

    // Find the point within the threshold by taking dot product
	for(i=0; i<nWindPoints; i++)
	{
		residuals = abs( windPoints[i].x*lnormal.x + windPoints[i].y*lnormal.y );
		if(residuals < sigmaFindSupport)
		{
			rpoints[nRindices] = windPoints[i];
			nRindices++;
		}
	}
	ldirection.x = -lnormal.y;
	ldirection.y = lnormal.x;

	if(nRindices<minLength_)
	{
		ls.nSupport_ = -1;
		return;
	}

	// Project to the line
	for(i=0; i<nRindices; i++)
	{
		rProjection[i] = rpoints[i].x*ldirection.x+ rpoints[i].y*ldirection.y;
		idx[i] = i;
	}
	
	// Sort the projection and find the starting and ending points	
	ISort(rProjection, nRindices, idx);

	for(i=0; i<nRindices; i++)
		absRProjection[i] = abs(rProjection[i]);

	for(i=0; i<nRindices; i++)
	{
		if(absRProjection[i] == 0)
		{
			zeroIndex = i;
			break;
		}
	}
	
	int maxIndex = nRindices-1;
	for(i=zeroIndex; i<(nRindices-1); i++)
	{	
		if((rProjection[i+1]-rProjection[i]) > maxGap)
		{	
			maxIndex = i;
			break;
		}
	}

	int minIndex = 0;
	for(i=zeroIndex; i>0; i--)
	{	
		if((rProjection[i]-rProjection[i-1]) > maxGap)
		{	
			minIndex = i;
			break;
		}
	}

    ls.nSupport_ = maxIndex - minIndex + 1;
	ls.sx_ = (double)rpoints[ idx[minIndex] ].x+x0;
	ls.sy_ = (double)rpoints[ idx[minIndex] ].y+y0;
	ls.ex_ = (double)rpoints[ idx[maxIndex] ].x+x0;
	ls.ey_ = (double)rpoints[ idx[maxIndex] ].y+y0;
	
	j = 0;
	for(i=minIndex; i<=maxIndex; i++)
	{
		proposedKillingList[j].x = rpoints[ idx[i] ].x + x0;
		proposedKillingList[j].y = rpoints[ idx[i] ].y + y0;
		j++;
	}
	nProposedKillingList = j;

	ls.normal_ = lnormal;

    delete [] rpoints;
    delete [] rProjection;
    delete [] absRProjection;
    delete [] idx;
#endif
}


//int LFLineFitter::FitALine(const int nWindPoints,Point<int> *windPoints,const double sigmaFitALine,Point<double> &lnormal)
//{
//	double inlierRatio = 0.9;	
//	double outlierRatio = 0.9;
//	double gamma = 0.05;	
//	int nMaxTry = 29; //ceil(log(0.05)/log(0.9))
//
//	int i=0,j=0,index=0;
//	int cscore;
//	double tmpScore;
//	double norm;
//	int bestscore = -1;	
//	Point<double> cdirection,cnormal;
//
//	while(i<nMaxTry)
//	{
//		index = (int)floor(rand()/(double)RAND_MAX*(double)nWindPoints);
//		norm = sqrt( 1.0*windPoints[index].x*windPoints[index].x + windPoints[index].y*windPoints[index].y );
//		
//		if(norm>0)
//		{
//			cdirection.x = windPoints[index].x/norm;
//			cdirection.y = windPoints[index].y/norm;
//
//			cnormal.x = -cdirection.y;
//			cnormal.y = cdirection.x;
//			
//			cscore = 0;
//			for(j=0;j<nWindPoints;j++)
//			{
//				tmpScore = abs(windPoints[j].x*cnormal.x+windPoints[j].y*cnormal.y);
//				if(tmpScore<sigmaFitALine)
//					cscore++;
//			}
//
//			if( ((double)cscore)/nWindPoints > inlierRatio)
//			{
//				bestscore = cscore;
//				lnormal = cnormal;
//				return bestscore;
//			}
//
//			if( (1.0-((double)cscore)/nWindPoints) < outlierRatio)
//			{
//				outlierRatio = 1.0-((double)cscore)/nWindPoints;
//				nMaxTry = (int)ceil(log(gamma)/log(outlierRatio));
//			}
//
//			if(cscore>bestscore)
//			{
//				bestscore = cscore;
//				lnormal = cnormal;
//			}
//		}
//		i = i+1;
//	}
//
//	return bestscore;
//}

int LFLineFitter::FitALine(const int nWindPoints,CvPoint *windPoints,const double sigmaFitALine,CvPoint2D64f &lnormal)
{
	double inlierRatio = 0.9;	
	double outlierRatio = 0.9;
	double gamma = 0.05;	
	int nMaxTry = 29; //ceil(log(0.05)/log(0.9))

	int i=0,j=0,index=0;
	int cscore;
	double tmpScore;
	double norm;
	int bestscore = -1;	
	CvPoint2D64f cdirection,cnormal;

	while(i<nMaxTry)
	{
		index = (int)floor(rand()/(double)RAND_MAX*(double)nWindPoints);
		norm = sqrt( 1.0*windPoints[index].x*windPoints[index].x + windPoints[index].y*windPoints[index].y );
		
		if(norm>0)
		{
			cdirection.x = windPoints[index].x/norm;
			cdirection.y = windPoints[index].y/norm;

			cnormal.x = -cdirection.y;
			cnormal.y = cdirection.x;
			
			cscore = 0;
			for(j=0;j<nWindPoints;j++)
			{
				tmpScore = abs(windPoints[j].x*cnormal.x+windPoints[j].y*cnormal.y);
				if(tmpScore<sigmaFitALine)
					cscore++;
			}

			if( ((double)cscore)/nWindPoints > inlierRatio)
			{
				bestscore = cscore;
				lnormal = cnormal;
				return bestscore;
			}

			if( (1.0-((double)cscore)/nWindPoints) < outlierRatio)
			{
				outlierRatio = 1.0-((double)cscore)/nWindPoints;
				nMaxTry = (int)ceil(log(gamma)/log(outlierRatio));
			}

			if(cscore>bestscore)
			{
				bestscore = cscore;
				lnormal = cnormal;
			}
		}
		i = i+1;
	}

	return bestscore;
}

//void LFLineFitter::Find(int x0,int y0,Point<int> *windPoints,int &nWindPoints,Image<unsigned char> *inputImage,int localWindSize)
//{
//	int x,y;
//	nWindPoints = 0;
//
//	for(y=max(y0-localWindSize,0);y<min(y0+localWindSize,inputImage->height());y++)
//		for(x=max(x0-localWindSize,0);x<min(x0+localWindSize,inputImage->width());x++)
//		{
//			//if(cvGetReal2D(inputImage,y,x)!=0)
//			if(imRef(inputImage,x,y)!=0)
//			{
//				windPoints[nWindPoints].x = x - x0;
//				windPoints[nWindPoints].y = y - y0;
//				nWindPoints++;
//			}
//		}
//}

void LFLineFitter::Find(int x0,int y0,CvPoint *windPoints,int &nWindPoints,IplImage *inputImage,int localWindSize)
{
	nWindPoints = 0;

	for(int y=max(y0-localWindSize,0);y<min(y0+localWindSize,inputImage->height);y++)
		for(int x=max(x0-localWindSize,0);x<min(x0+localWindSize,inputImage->width);x++)
		{
			if(cvGetReal2D(inputImage,y,x)!=0)
			{
				windPoints[nWindPoints].x = x - x0;
				windPoints[nWindPoints].y = y - y0;
				nWindPoints++;
			}
		}
}


void LFLineFitter::Find(map<int,Point<int> > *edgeMap,int x0,int y0,Point<int> *windPoints,int &nWindPoints,Image<unsigned char> *inputImage,int localWindSize)
{
	nWindPoints = 0;
	map<int,Point<int> >::iterator it;

	int left = max(x0-localWindSize,0);
	int right = min(x0+localWindSize,inputImage->width());
	int top = max(y0-localWindSize,0);
	int bottom = min(y0+localWindSize,inputImage->height());

	for(it = edgeMap->begin();it!=edgeMap->end();++it)
	{
		if( it->second.x > left && it->second.x < right && 
			it->second.y > top && it->second.y < bottom )
		{
			windPoints[nWindPoints].x = it->second.x - x0;
			windPoints[nWindPoints].y = it->second.y - y0;
			nWindPoints++;
		}
	}
}

void LFLineFitter::Find(map<int,CvPoint> *edgeMap,int x0,int y0,CvPoint *windPoints,int &nWindPoints,IplImage *inputImage,int localWindSize)
{
	nWindPoints = 0;
	map<int,CvPoint>::iterator it;

	int left = max(x0-localWindSize,0);
	int right = min(x0+localWindSize,inputImage->width);
	int top = max(y0-localWindSize,0);
	int bottom = min(y0+localWindSize,inputImage->height);

	for(it = edgeMap->begin();it!=edgeMap->end();++it)
	{
		if( it->second.x > left && it->second.x < right && 
			it->second.y > top && it->second.y < bottom )
		{
			windPoints[nWindPoints].x = it->second.x - x0;
			windPoints[nWindPoints].y = it->second.y - y0;
			nWindPoints++;
		}
	}
}

int LFLineFitter::SampleAPixel(map<int,Point<int> > *edgeMap,Image<unsigned char> *inputImage,int nPixels)
{
	int index = (int)(floor(rand()/(double)RAND_MAX*(double)(edgeMap->size()-1)));
	map<int,Point<int> >::iterator it;
	it=edgeMap->begin();
	for(int i=0;i<index;i++)
		it++;
	return (*it).first;
}

int LFLineFitter::SampleAPixel(map<int,CvPoint> *edgeMap,IplImage *inputImage,int nPixels)
{
	int index = (int)(floor(rand()/(double)RAND_MAX*(double)(edgeMap->size()-1)));
	map<int,CvPoint>::iterator it;
	it=edgeMap->begin();
	for(int i=0;i<index;i++)
		it++;
	return (*it).first;
}

void LFLineFitter::Configure(const char *filename)
{
	//ifstream file;
	//file.open(filename);
	//string line;
	//string column(":");
	//size_t found;

	//if( !file.is_open() )
	//{
	//	cerr<<"Cannot open file "<<filename<<endl;
	//	exit(-1);
	//}

	//while( getline(file,line) )
	//{
	//	if( string::npos != line.find("SIGMA_FIT_A_LINE") )
	//	{
	//		found = line.find(column);
	//		string substring = line.substr(found+1,line.length()-found).c_str();
	//		sigmaFitALine_ = atof(substring.c_str());
	//	}
	//	else if(string::npos != line.find("SIGMA_FIND_SUPPORT") )
	//	{
	//		found = line.find(column);
	//		string substring = line.substr(found+1,line.length()-found).c_str();
	//		sigmaFindSupport_ = atof(substring.c_str());
	//	}
	//	else if(string::npos != line.find("MAX_GAP:") )
	//	{
	//		found = line.find(column);
	//		string substring = line.substr(found+1,line.length()-found).c_str();
	//		maxGap_ = atof(substring.c_str());
	//	}
	//	else if(string::npos != line.find("N_LINES_TO_FIT_IN_STAGE_1") )
	//	{
	//		found = line.find(column);
	//		string substring = line.substr(found+1,line.length()-found).c_str();
	//		nLinesToFitInStage_[0] = atoi(substring.c_str());
	//	}
	//	else if(string::npos != line.find("N_TRIALS_PER_LINE_IN_STAGE_1") )
	//	{
	//		found = line.find(column);
	//		string substring = line.substr(found+1,line.length()-found).c_str();
	//		nTrialsPerLineInStage_[0] = atoi(substring.c_str());
	//	}
	//	else if(string::npos != line.find("N_LINES_TO_FIT_IN_STAGE_2") )
	//	{
	//		found = line.find(column);
	//		string substring = line.substr(found+1,line.length()-found).c_str();
	//		nLinesToFitInStage_[1] = atoi(substring.c_str());
	//	}
	//	else if(string::npos != line.find("N_TRIALS_PER_LINE_IN_STAGE_2") )
	//	{
	//		found = line.find(column);
	//		string substring = line.substr(found+1,line.length()-found).c_str();
	//		nTrialsPerLineInStage_[1] = atoi(substring.c_str());
	//	}
	//}
	//file.close();
    //
	//PrintParameter();


    for(int i=0;i<50;i++)
		cout<<"*";
	cout<<endl;

	cout<<" LFLineFitting"<<endl;

	for(int i=0;i<50;i++)
		cout<<"*";
	cout<<endl;


	char str[256];
	FILE *fp=NULL;
	fp = fopen(filename,"rt");
	if(fp==NULL)
	{
		cerr<<"[ERROR] Cannot read file "<<filename<<"\n!!!";
		exit(0);
	}
	
	cout<<"Load Configuration from "<<filename<<endl;
	
	fscanf(fp,"%s\n",str);
	cout<<str<<" = ";
	fscanf(fp,"%s\n",str);
	sigmaFitALine_ = atof(str);
	cout<<sigmaFitALine_<<endl;

	fscanf(fp,"%s\n",str);
	cout<<str<<" = ";
	fscanf(fp,"%s\n",str);
	sigmaFindSupport_ = atof(str);
	cout<<sigmaFindSupport_<<endl;

	fscanf(fp,"%s\n",str);
	cout<<str<<" = ";
	fscanf(fp,"%s\n",str);
	maxGap_ = atof(str);
	cout<<maxGap_<<endl;

	for(int i=0;i<2;i++)
	{
		fscanf(fp,"%s\n",str);
		cout<<str<<" = ";
		fscanf(fp,"%s\n",str);
		nLinesToFitInStage_[i] = (int)atof(str);
		cout<<nLinesToFitInStage_[i]<<endl;

		fscanf(fp,"%s\n",str);
		cout<<str<<" = ";
		fscanf(fp,"%s\n",str);
		nTrialsPerLineInStage_[i] = (int)atof(str);
		cout<<nTrialsPerLineInStage_[i]<<endl;
	}

	fclose(fp);
	cout<<endl<<endl;
}


void LFLineFitter::PrintParameter()
{
	cout<<"/* =========================================================="<<endl;
	cout<<"* LFLineFitting parameters "<<endl;
	cout<<"* ==========================================================="<<endl;
	cout<<"* SIGMA_FIT_A_LINE:"<<"\t"<<sigmaFitALine_<<endl;
	cout<<"* SIGMA_FIND_SUPPORT:"<<"\t"<<sigmaFindSupport_<<endl;
	cout<<"* MAX_GAP:"<<"\t\t"<<maxGap_<<endl;
	cout<<"* N_LINES_TO_FIT_IN_STAGE_1:"<<"\t"<<nLinesToFitInStage_[0]<<endl;
	cout<<"* N_TRIALS_PER_LINE_IN_STAGE_1:"<<"\t"<<nTrialsPerLineInStage_[0]<<endl;
	cout<<"* N_LINES_TO_FIT_IN_STAGE_2:"<<"\t"<<nLinesToFitInStage_[1]<<endl;
	cout<<"* N_TRIALS_PER_LINE_IN_STAGE_2:"<<"\t"<<nTrialsPerLineInStage_[1]<<endl;	
	cout<<"* ==========================================================="<<endl;
	cout<<"*/"<<endl;
}

void LFLineFitter::ISort(double* ra, int nVec, int* ira)
{
   unsigned long n, l, ir, i, j;
   n = nVec;
   double rra;
   int irra;
   
   if (n<2)
      return;
   l = (n>>1)+1;
   ir = n;
   for (;;)
   {
      if (l>1)
      {
         irra = ira[(--l)-1];
         rra = ra[l-1];
      }
      else
      {
         irra = ira[ir-1];
         rra = ra[ir-1];

         ira[ir-1] = ira[1-1];
         ra[ir-1] = ra[1-1];

         if (--ir==1)
         {
            ira[1-1] = irra;
            ra[1-1] = rra;
            break;
         }
      }
      i = l;
      j = l+l;
      while (j<=ir)
      {
         if (j<ir && ra[j-1]<ra[j+1-1])
            j++;
         if (rra<ra[j-1])
         {
            ira[i-1] = ira[j-1];
            ra[i-1] = ra[j-1];

            i = j;
            j <<= 1;
         }
         else
            j = ir+1;
      }
      ira[i-1] = irra;
      ra[i-1] = rra;
   }

}
