#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <amino.h>
#include <ach.h>
#include <sns.h>

#include <cstdio>
#include <iostream>

#define DEBUG false

using namespace cv;
using namespace std;

typedef struct {
    size_t width;
    size_t height;
    char points[1][3];
} frame_t;

class ImageReceiver
{
public:
	ImageReceiver() {initialized = false;}
	void Init(const char* channelName, int resX, int resY )
	{
	    sns_init();
	    int r = ach_open(&chan, channelName, NULL);
	    assert( ACH_OK == r );
   
	    iWidth  = resX;
	    iHeight = resY;

	    size = sizeof(frame) + sizeof(frame->points[0])*(iWidth*iHeight*3);
	    frame = (frame_t*) (malloc(size));
	    frame->width = iWidth;
	    frame->height = iHeight;
	    initialized = true;
	}

	Mat receiveImage()
	{
		Mat image(frame->width, frame->height, CV_8UC3, Scalar(0,0,255));
		if(!initialized) return image;

		size_t fs;
		if (DEBUG)
			std::cout << "Pull message .." << std::endl; 	
		
		ach_status_t r = ach_get( &chan, frame, size, &fs,
		             NULL, (ach_get_opts_t)(ACH_O_LAST) );
		
		int n = frame->width * frame->height;
		//assert( n < fs / sizeof(frame->points[0][0]) )
		
		int i = 0;
		for(int x = 0; x < frame->width; x++)
		{
			for(int y = 0; y < frame->height; y++)
			{
		    		int r = (int)frame->points[i][0];
		    		int g = (int)frame->points[i][1];
		    		int b = (int)frame->points[i][2];
						if(DEBUG)
							std::cout << r << " " << g << " " << b << std::endl;
		    		assert(i < frame->width*frame->height);
				image.at<Vec3b>(x,y)[0] = r;
				image.at<Vec3b>(x,y)[1] = g;
				image.at<Vec3b>(x,y)[2] = b;
				i++;
			}
		}
		return image;
	}

private:
	int size, iHeight, iWidth;
	ach_channel_t chan;
	frame_t* frame;
	bool initialized;
};






