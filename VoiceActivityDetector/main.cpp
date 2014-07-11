/** @file paex_record.c
	@ingroup examples_src
	@brief Record input into an array; Save array to a file; Playback recorded data.
	@author Phil Burk  http://www.softsynth.com
*/
/*
 * $Id: paex_record.c 1752 2011-09-08 03:21:55Z philburk $
 *
 * This program uses the PortAudio Portable Audio Library.
 * For more information see: http://www.portaudio.com
 * Copyright (c) 1999-2000 Ross Bencina and Phil Burk
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * The text above constitutes the entire PortAudio license; however, 
 * the PortAudio community also makes the following non-binding requests:
 *
 * Any person wishing to distribute modifications to the Software is
 * requested to send the modifications to the original developer so that
 * they can be incorporated into the canonical version. It is also 
 * requested that these non-binding requests be included along with the 
 * license above.
 */

/* Standrad Included Library */
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Third-party Library */
	// For audio processing
#include "portaudio.h"
	// LCM core
#include "lcm\lcm-cpp.hpp"
	// LCM shared consts
#include "lcm\LcmComm.hpp"
	// LCM message data type
#include "lcm\VoiceDetectorLcm.hpp"

using namespace std;
using namespace lcm;

/* #define SAMPLE_RATE  (17932) // Test failure to open with this value. */
#define SAMPLE_RATE			(8000)
#define FRAMES_PER_BUFFER	(1024)
#define NUM_SECONDS			(1)
#define NUM_CHANNELS		(1)
#define VOICE_THRESH_LOW	(25)//(50)
#define VOICE_THRESH_MEDIUM	(75)//(150)
#define VOICE_THRESH_HIGH	(150)//(300)
//#define VOICE_THRESH		(0.01)
	// How long a timestep is (ms)
#define STEPTIME			(250)
/* #define DITHER_FLAG     (paDitherOff) */
#define DITHER_FLAG			(0) /**/
/** Set to 1 if you want to capture the recording to a file. */
#define WRITE_TO_FILE		(0)

/* Select sample format. */
#define PA_SAMPLE_TYPE  paFloat32
typedef float SAMPLE;
#define SAMPLE_SILENCE  (0.0f)
#define PRINTF_S_FORMAT "%.8f"

typedef struct
{
	int          frameIndex;  /* Index into sample array. */
	int          maxFrameIndex;
	SAMPLE      *recordedSamples;
}
paTestData;

paTestData data; // create an global variable to share audio track

/* This routine will be called by the PortAudio engine when audio is needed.
** It may be called at interrupt level on some machines so don't do anything
** that could mess up the system like calling malloc() or free().
*/
static int recordCallback( const void *inputBuffer, void *outputBuffer,
						   unsigned long framesPerBuffer,
						   const PaStreamCallbackTimeInfo* timeInfo,
						   PaStreamCallbackFlags statusFlags,
						   void *userData )
{
	paTestData *data = (paTestData*)userData;
	const SAMPLE *rptr = (const SAMPLE*)inputBuffer;
	SAMPLE *wptr = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
	long framesToCalc;
	long i;
	int finished;
	unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;

	(void) outputBuffer; /* Prevent unused variable warnings. */
	(void) timeInfo;
	(void) statusFlags;
	(void) userData;

	if( framesLeft < framesPerBuffer )
	{
		framesToCalc = framesLeft;
		finished = paComplete;
	}
	else
	{
		framesToCalc = framesPerBuffer;
		finished = paContinue;
	}

	if( inputBuffer == NULL )
	{
		for( i=0; i<framesToCalc; i++ )
		{
			*wptr++ = SAMPLE_SILENCE;  /* left */
			if( NUM_CHANNELS == 2 ) *wptr++ = SAMPLE_SILENCE;  /* right */
		}
	}
	else
	{
		for( i=0; i<framesToCalc; i++ )
		{
			*wptr++ = *rptr++;  /* left */
			if( NUM_CHANNELS == 2 ) *wptr++ = *rptr++;  /* right */
		}
	}
	data->frameIndex += framesToCalc;
	return finished;
}

/*******************************************************************/
int isVoice()
{
	int nowFrameIndex;
	int startFrameIndex;
	float sumSample;
	//float sumThresh;
	int i;

	sumSample = 0.0;
	nowFrameIndex = data.frameIndex;
	startFrameIndex = (nowFrameIndex - SAMPLE_RATE * STEPTIME / 1000);

	if(startFrameIndex < 0)
		startFrameIndex = 0;

	SAMPLE *rptr = &data.recordedSamples[startFrameIndex * NUM_CHANNELS];
	for( i=startFrameIndex; i<nowFrameIndex; i++)
	{
		sumSample += fabs((*rptr++));
		if(NUM_CHANNELS==2)
			sumSample += fabs((*rptr++));
	}

	//sumThresh = VOICE_THRESH*(float)NUM_CHANNELS*(float)(nowFrameIndex-startFrameIndex);
	//sumThresh = VOICE_THRESH;
	printf("> SumSample= %f\n", sumSample);

	if (sumSample > VOICE_THRESH_HIGH)
		return 3;
	else if (sumSample > VOICE_THRESH_MEDIUM)
		return 2;
	else if (sumSample > VOICE_THRESH_LOW)
		return 1;
	else
		return 0;
}

/*******************************************************************/
int main (void) {
	PaStreamParameters  inputParameters;
						//outputParameters;
	PaStream*           stream;
	PaError             err = paNoError;
	//paTestData          data; // Declare in global using for whole program
	int                 i;
	int                 totalFrames;
	int                 numSamples;
	int                 numBytes;
	SAMPLE              max, val;
	double              average;
	int timeCount = 0;

	printf("patest_record.c\n"); fflush(stdout);

	/* Initialize LCM */
	LCM lcm(LCM_CTOR_PARAMS);
	if (!lcm.good()) 
	{
		printf("> ERROR: Cannot initialize LCM.\n");
		goto done;
	}

	/* Proprecessing data structure */
	data.maxFrameIndex = totalFrames = NUM_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
	data.frameIndex = 0;
	numSamples = totalFrames * NUM_CHANNELS;
	numBytes = numSamples * sizeof(SAMPLE);
	data.recordedSamples = (SAMPLE *) malloc( numBytes ); /* From now on, recordedSamples is initialised. */
	if( data.recordedSamples == NULL )
	{
		printf("Could not allocate record array.\n");
		goto done;
	}
	for( i=0; i<numSamples; i++ )
		data.recordedSamples[i] = 0;

	/* Initialize PortAudio */
	err = Pa_Initialize();
	if( err != paNoError ) goto done;

	inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
	if (inputParameters.device == paNoDevice) {
		fprintf(stderr,"Error: No default input device.\n");
		goto done;
	}
	inputParameters.channelCount = NUM_CHANNELS;                    /* stereo input */
	inputParameters.sampleFormat = PA_SAMPLE_TYPE;
	inputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
	inputParameters.hostApiSpecificStreamInfo = NULL;

	/* Record some audio. -------------------------------------------- */
	err = Pa_OpenStream(
			  &stream,
			  &inputParameters,
			  NULL,                  /* &outputParameters, */
			  SAMPLE_RATE,
			  FRAMES_PER_BUFFER,
			  paClipOff,      /* we won't output out of range samples so don't bother clipping them */
			  recordCallback,
			  &data );
	if( err != paNoError ) goto done;

	err = Pa_StartStream( stream );
	if( err != paNoError ) goto done;

	/* Start collecting the audio input */
	printf("\n> ---------- Now recording ----------\n"); fflush(stdout);
	int sampleRate = STEPTIME + 15;
	while( ( err = Pa_IsStreamActive( stream ) ) == 1 )
	{
		Pa_Sleep(sampleRate);

		/* Send message through LCM */
		VoiceDetectorLcm voiceMgr;
		int VoiceFlag = isVoice();
		voiceMgr.voice_detection = VoiceFlag;
		lcm.publish(VOICE_DETECTION, &voiceMgr);
		printf("> Second = %d | Data Index = %d | Sending... (VAD: %d) \n", timeCount++, data.frameIndex, VoiceFlag); fflush(stdout);

		/* Reset the frameIndex to avoid the termination of the audio stream */
		//int bufferSize = SAMPLE_RATE * STEPTIME / 1000;
		//SAMPLE *rptr = &data.recordedSamples[data.frameIndex - bufferSize * NUM_CHANNELS];
		//for (i = 0; i < bufferSize; ) {
		//	rptr[i++] = (*rptr++);
		//	if(NUM_CHANNELS == 2)
		//		rptr[i++] = (*rptr++);
		//}
		//data.frameIndex = bufferSize;
		data.frameIndex = 0;
	}
	if( err < 0 )
		goto done;

	err = Pa_CloseStream( stream );
	if( err != paNoError )
		goto done;

	/* Measure maximum peak amplitude. */
	max = 0;
	average = 0.0;
	for( i=0; i<numSamples; i++ )
	{
		val = data.recordedSamples[i];
		if( val < 0 ) val = -val; /* ABS */
		if( val > max )
		{
			max = val;
		}
		average += val;
	}

	average = average / (double)numSamples;

	printf("sample max amplitude = "PRINTF_S_FORMAT"\n", max );
	printf("sample average = %lf\n", average );
	
done:
	Pa_Terminate();
	if( data.recordedSamples )       /* Sure it is NULL or valid. */
		free( data.recordedSamples );
	if( err != paNoError )
	{
		fprintf( stderr, "An error occured while using the portaudio stream\n" );
		fprintf( stderr, "Error number: %d\n", err );
		fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
		err = 1;          /* Always return 0 or 1, but no other return codes. */
	}
	//disconnect_to_server();
	return err;
}