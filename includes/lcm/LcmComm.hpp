#ifndef LCMCOMM_HPP
#define LCMCOMM_HPP

// Parameter for LCM constructor. Must the same for all programs to communicate properly.
const char* LCM_CTOR_PARAMS = "udpm://239.255.76.67:7667?ttl=1";

// Channel names
const char* FACE_DETECTION		= "FACE_DETECTION";
const char* BODY_DIRECTION		= "BODY_DIRECTION";
const char* VOICE_DETECTION		= "VOICE_DETECTION";
const char* LEG_CHANNEL			= "PEOPLE";

#endif