// The codes in this page is derived from samples provided by Microsoft.
// Kinect2DepthBasics.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "kinect2depthbasics.h"
#include "kinect2/Kinect.h"
#include <thread>

bool ExitThreadMsg;
bool DepthReady;
bool BodyReady;
bool NewFrameReady;

IKinectSensor*          m_pKinectSensor;
IDepthFrameReader*      m_pDepthFrameReader;
IBodyFrameReader*       m_pBodyFrameReader;
ICoordinateMapper*      m_pCoordinateMapper;

Kinect2DepthFrame frame;
Kinect2DepthFrame output;
unsigned short* framedata = NULL;

void ThreadDepth()
{
	while (!ExitThreadMsg)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		IDepthFrame* pDepthFrame = NULL;
		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTimeDepth = 0;
			hr = pDepthFrame->get_RelativeTime(&nTimeDepth);
			IFrameDescription* pFrameDescription = NULL;

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			}
			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&frame.Width);
			}
			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Height(&frame.Height);
			}
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMinReliableDistance(&frame.DepthMinReliableDistance);
			}
			if (SUCCEEDED(hr))
			{
				// In order to see the full range of depth (including the less reliable far field depth)
				// we are setting nDepthMaxDistance to the extreme potential depth threshold
				frame.DepthMaxDistance = USHRT_MAX;

				// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
				//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
			}
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&frame.BufferSize, &frame.pBuffer);
			}
			if (SUCCEEDED(hr))
			{
				if (framedata == NULL)
					framedata = new unsigned short[frame.BufferSize];
				memcpy(framedata, frame.pBuffer, frame.BufferSize * 2);
				DepthReady = true;
			}

			pFrameDescription->Release();
			pDepthFrame->Release();
		}
		else
		{
		}
	}
}

void ThreadBody()
{
	while (!ExitThreadMsg)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		IBodyFrame* pBodyFrame = NULL;
		IBody* ppBodies[BODY_COUNT] = { 0 };
		BOOLEAN bTracked = false;
		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hr))
		{
			INT64 nTimeBody = 0;
			hr = pBodyFrame->get_RelativeTime(&nTimeBody);

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}
			if (SUCCEEDED(hr))
			{
				IBody* pBody = NULL;
				for (int b = 0; b < BODY_COUNT; b++)
				{
					pBody = ppBodies[b];
					pBody->get_IsTracked(&bTracked);
					if (bTracked)
						break;
				} 
				if (bTracked)
				{
					Joint joints[JointType_Count];
					hr = pBody->GetJoints(_countof(joints), joints);

					DepthSpacePoint depthPoint = { 0 };
					m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_HandRight].Position, &depthPoint);
					frame.RightHandY = depthPoint.X;
					frame.RightHandX = depthPoint.Y;
					m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_ElbowRight].Position, &depthPoint);
					frame.RightElbowX = depthPoint.X;
					frame.RightElbowY = depthPoint.Y;

					BodyReady = true;
				}
			}

			pBodyFrame->Release();
			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				ppBodies[i]->Release();
			}
		}
		else
		{
		}
	}
	return;
}

int InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		IDepthFrameSource* pDepthFrameSource = NULL;
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		pDepthFrameSource->Release();
		pBodyFrameSource->Release();
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		//SetStatusMessage(L"No ready Kinect found!", 10000, true);
		return E_FAIL;
	}

	return hr;
}

int kinect2_start()
{
	HRESULT hr = InitializeDefaultSensor();
	if (FAILED(hr))
		return hr;

	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	DepthReady = false;
	BodyReady = false;
	NewFrameReady = false;
	ExitThreadMsg = false;
	new std::thread(ThreadDepth);
	new std::thread(ThreadBody);
	return 0;
}

int kinect2_stop()
{
	ExitThreadMsg = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	m_pDepthFrameReader->Release();
	m_pDepthFrameReader = NULL;

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	m_pKinectSensor->Release();
	m_pKinectSensor = NULL;
	return 0;
}

Kinect2DepthFrame* kinect2_getframesync()
{
	while (!DepthReady || !BodyReady)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	DepthReady = false;
	BodyReady = false;

	output = frame;
	output.pBuffer = framedata;

	return &output;
}

