
#include "Thread.h"

#ifdef WIN32
#include <iostream>
#include <process.h>
#endif

#include <cassert>

using namespace ROBOT;

unsigned int Thread::threadCount = 0;

Thread::Thread() : runnable(NULL), runningFlag(false), stopFlag(false)
{
#ifdef WIN32
	hThread = 
		(HANDLE)_beginthreadex(NULL,0,Thread::startThread, (LPVOID)this, CREATE_SUSPENDED, &threadID);
	if(!hThread)
		printError( (LPSTR)"_beginthreadex failed at ",__FILE__, __LINE__);
	else
		threadCount++;
#else
	suspend_mutex = PTHREAD_MUTEX_INITIALIZER;
	suspend_cond = PTHREAD_COND_INITIALIZER;
	threadReady = 0;

	pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	if( pthread_create(&threadID, &attr, startThread, this)!=0 )
		;
	else
		threadCount++;
	pthread_attr_destroy(&attr);
#endif
}

Thread::Thread(std::auto_ptr<Runnable> r) : runnable(r), runningFlag(false), stopFlag(false)
{
#ifdef WIN32
	if(!runnable.get())
		printError( (LPSTR)"Thread(std::auto_ptr<Runnable> r) failed at ", __FILE__, __LINE__);
	hThread = 
		(HANDLE)_beginthreadex(NULL,0,Thread::startThread, (LPVOID)this, CREATE_SUSPENDED, &threadID);
	if(!hThread)
		printError( (LPSTR)"_beginthreadex failed at ", __FILE__, __LINE__);
	else
		threadCount++;
#else
	suspend_mutex = PTHREAD_MUTEX_INITIALIZER;
	suspend_cond = PTHREAD_COND_INITIALIZER;
	threadReady = 0;

	pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	if( pthread_create(&threadID, &attr, startThread, this)!=0 )
		;
	else
		threadCount++;
	pthread_attr_destroy(&attr);
#endif
}

Thread::~Thread() 
{
#ifdef WIN32
	if(threadID != GetCurrentThreadId()) {
		stop();
		join();
		DWORD rc = CloseHandle(hThread);
		if(!rc) printError
			( (LPSTR)"CloseHandle failed at ",__FILE__, __LINE__);
	}
#else
	// TODO
	if( threadID != pthread_self() ){
		if( isRunning() ){
			stop();
		}
		join();
	}
#endif
	threadCount--;
}

void Thread::start() 
{
	stopFlag = false;

#ifdef WIN32
	assert(hThread);
	DWORD rc = ResumeThread(hThread);
	if(!rc) printError
			( (LPSTR)"ResumeThread failed at ",__FILE__, __LINE__);
#else
	pthread_mutex_lock(&suspend_mutex);
	threadReady = 1;
	pthread_mutex_unlock(&suspend_mutex);
	pthread_cond_signal(&suspend_cond);
#endif
}

void Thread::stop()
{
	/*
#ifdef WIN32
	stopFlag = true;
#else
	int rc = pthread_cancel(threadID);
	printError( "pthread_cancel() : ", rc );
#endif
	*/
	stopFlag = true;
}

bool Thread::isRunning() const
{
	return runningFlag;
}

void* Thread::join() 
{
#ifdef WIN32
	//SetEvent(hThread);
	WaitForSingleObject( hThread, INFINITE );
#else
	pthread_join(threadID,0);
#endif
	return result;
}

unsigned int Thread::count()
{
	return threadCount;
}

void Thread::sleep(unsigned int milli)
{
#ifdef WIN32
    Sleep(milli);
#else
   struct timespec ts = { milli/1000, (milli%1000)*1000000 };
   nanosleep(&ts, 0);
#endif
}

void Thread::yield()
{
#ifdef WIN32
	Sleep(0);
#else
	pthread_yield();
#endif
}

bool Thread::shouldStop()
{
	/*
#ifdef WIN32
	if( stopFlag ){
		_endthreadex(0);
	}
#else
	pthread_testcancel();
#endif*/
	return stopFlag;
}

#ifdef WIN32
unsigned WINAPI Thread::startThread(LPVOID pVoid)
#else
void* Thread::startThread(void* pVoid)
#endif
{
	Thread* aThread = static_cast<Thread*>(pVoid);
#ifndef WIN32
	pthread_mutex_lock(&aThread->suspend_mutex);
	while (aThread->threadReady == 0) {
		pthread_cond_wait(&aThread->suspend_cond, &aThread->suspend_mutex);
	}
	pthread_mutex_unlock(&aThread->suspend_mutex);
#endif
	aThread->runningFlag = true;
	if(aThread->runnable.get() ){
		aThread->result = aThread->runnable->run();
	}
	else{
		aThread->result = aThread->run();
	}
	aThread->runningFlag = false;
#ifdef WIN32
	return reinterpret_cast<unsigned>(aThread->result);
#else
	return reinterpret_cast<void*>(aThread->result);
#endif
}


#ifdef WIN32
void Thread::printError(LPSTR lpszFunction, LPSTR fileName, int lineNumber)
{
	TCHAR szBuf[256];
	LPSTR lpErrorBuf;
	DWORD errorCode=GetLastError();
	FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER||
		FORMAT_MESSAGE_FROM_SYSTEM,
		NULL,
		errorCode,
		MAKELANGID(LANG_NEUTRAL,SUBLANG_DEFAULT),
		(LPTSTR)&lpErrorBuf,
		0,
		NULL);
	wsprintfA( (LPSTR)szBuf,(LPCSTR)"%s failed at line %d in %s with error %d: %s", 
		     lpszFunction, lineNumber, fileName, errorCode, lpErrorBuf);
	DWORD numWritten; 
	WriteFile(GetStdHandle(STD_ERROR_HANDLE),
		szBuf,
		strlen(reinterpret_cast <const char *> (szBuf)),
		&numWritten,
		FALSE);
	LocalFree(lpErrorBuf);
	exit(errorCode);
}

#else

void Thread::printError( char*, int rc )
{

}

#endif

void Thread::suspend() {
#ifdef WIN32

	SuspendThread(hThread);

#else



#endif
}