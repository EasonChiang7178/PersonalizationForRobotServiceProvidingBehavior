#ifndef LCMHANDLERS
#define LCMHANDLERS

#include <string>

#include "lcm\lcm-cpp.hpp"
#include "lcm\BodyDirectionLcm.hpp"
#include "lcm\FaceDetectionLcm.hpp"
#include "lcm\VoiceDetectorLcm.hpp"

class HAEHandlerLcm
{
public:
	~HAEHandlerLcm() {}
	
	void handleBody(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const BodyDirectionLcm* msg)
	{
		body_direction = msg->body_direction;
		pu = msg->pu;
		body_direction_cont = msg->body_direction_cont;
	}

	void handleFace(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const FaceDetectionLcm* msg)
	{
		face_direction = msg->face_direction;
	}

	void handleVoice(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const VoiceDetectorLcm* msg)
	{
		voice_detection = msg->voice_detection;
	}

	int face_direction;
	int voice_detection;
	int body_direction;
	int pu;
	float body_direction_cont;
};

#endif