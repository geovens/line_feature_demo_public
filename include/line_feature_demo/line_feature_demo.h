#pragma once
#include "opencv2/opencv.hpp"

class line_feature_demo_inner;
class line_feature_demo
{
public:
	line_feature_demo_inner* inner;

	__declspec(dllexport) int initialize();
	__declspec(dllexport) int classify(cv::Mat* frame, int right_hand_x, int right_hand_y);
	__declspec(dllexport) int show();
	__declspec(dllexport) int destroy();
};