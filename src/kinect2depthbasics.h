#pragma once

typedef struct
{
	int Width;
	int Height;
	unsigned short DepthMinReliableDistance;
	unsigned short DepthMaxDistance;
	unsigned int BufferSize;
	unsigned short* pBuffer;

	float RightHandY;
	float RightHandX;
	float RightElbowX;
	float RightElbowY;
} Kinect2DepthFrame;

int kinect2_start();
int kinect2_stop();
Kinect2DepthFrame* kinect2_getframesync();
