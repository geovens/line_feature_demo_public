// A demo program for paper Fast Hand Posture Classification Using Depth Features Extracted from Random Line Segments
// Weizhi Nai

#include "stdafx.h"
#include "line_feature_demo/line_feature_demo.h"
#include "kinect2depthbasics.h"

int _tmain(int argc, _TCHAR* argv[])
{
	line_feature_demo demo;
	demo.initialize();

	long rec = kinect2_start();
	if (rec < 0)
	{
		printf("Initializing Kinect 2 failed.\n");
		getchar();
		return 0;
	}
	printf("Waiting for recognition of body skeleton (please make a pose)...\n");

	cv::Mat* cv_frame = NULL;

	for (int f = 0; f < 18000; f++)
	{
		Kinect2DepthFrame* kinect2_frame = kinect2_getframesync();
		if (cv_frame == NULL)
		{
			cv_frame = new cv::Mat(kinect2_frame->Height, kinect2_frame->Width, CV_16UC1);
			cv_frame->cols = kinect2_frame->Width;
			cv_frame->rows = kinect2_frame->Height;
			printf("Body skeleton recognized.\n");
		}

		cv_frame->data = (unsigned char*)kinect2_frame->pBuffer;

		int label = demo.classify(cv_frame, kinect2_frame->RightHandX, kinect2_frame->RightHandY);

		demo.show();
	}

	kinect2_stop();

	return 0;
}

