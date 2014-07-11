#ifndef LCMHANDLERS
#define LCMHANDLERS

#include <string>

#include "lcm\lcm-cpp.hpp"
#include "lcm\BodyDirectionLcm.hpp"
#include "lcm\FaceDetectionLcm.hpp"
#include "lcm\VoiceDetectorLcm.hpp"
#include "lcm\LegDetectLcm.hpp"

#define DEBUG

class HAEHandlerLcm
{
public:
	~HAEHandlerLcm() {}
	
	void handleBody(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const BodyDirectionLcm* msg)
	{
#ifdef DEBUG
		cout << "> Received message on channel \""<< chan.c_str() << "\"" << endl;
#endif //DEBUG
		body_direction = msg->body_direction;
		pu = msg->pu;
		body_direction_cont = msg->body_direction_cont;
	}

	void handleFace(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const FaceDetectionLcm* msg)
	{
#ifdef DEBUG
		cout << "> Received message on channel \""<< chan.c_str() << "\"" << endl;
#endif //DEBUG
		face_direction = msg->face_direction;
	}

	void handleVoice(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const VoiceDetectorLcm* msg)
	{
#ifdef DEBUG
		cout << "> Received message on channel \""<< chan.c_str() << "\"" << endl;
#endif //DEBUG
		voice_detection = msg->voice_detection;
	}

	int face_direction;
	int voice_detection;
	int body_direction;
	int pu;
	float body_direction_cont;
};

class lcmLegDetect
{
public:
	~lcmLegDetect() {}

	void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LegDetectLcm* msg) {
#ifdef DEBUG
		cout << "> Received message on channel \""<< chan.c_str() << "\"" << endl;
#endif //DEBUG

		count = msg->count;
		for (int i(0); i < msg->count; ++i) {
			x[i] = msg->x[i];
			y[i] = msg->y[i];
			vel[i] = msg->vel[i];
			theta[i] = msg->theta[i];
		}
	}

	int32_t count;
	float x[10];
	float y[10];
	float vel[10];
	float theta[10];
};

#endif