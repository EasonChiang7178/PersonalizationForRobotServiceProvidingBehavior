#ifndef ROBOT_THREAD_H_
#define ROBOT_THREAD_H_

#include <memory>

#ifdef WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif

#include "Runnable.h"

namespace ROBOT {

class Thread {
public:
	Thread();
	Thread(std::auto_ptr<Runnable> run);	
	Thread(const Thread&); //
	virtual ~Thread();

	void start();
	void stop(); 
	bool isRunning() const;
	void suspend();

#ifdef WIN32
	unsigned getID(); //
#else
	pthread_t getID();
#endif

	void* join();
	const Thread& operator=(const Thread&);//

	static unsigned int count();

	static void sleep(unsigned int milli);
	static void yield();

protected:
	bool shouldStop();

private:
#ifdef WIN32
	HANDLE hThread;
	unsigned threadID;
#else
	pthread_t threadID;
#endif
	bool runningFlag;
	bool stopFlag;
	
	std::auto_ptr<Runnable> runnable; // runnable object will be deleted automatically by auto_ptr
	virtual void* run() {return 0;}

#ifndef WIN32
	pthread_mutex_t suspend_mutex;
	pthread_cond_t suspend_cond;
	int threadReady;
#endif

#ifdef WIN32
	static unsigned WINAPI startThread(LPVOID pVoid);
	void printError(LPSTR lpszFunction, LPSTR fileName, int lineNumber);
#else
	static void* startThread(void* pVoid);
	void printError( char*, int rc );
	static pthread_key_t ourKey;
#endif
	// stores return value from run()
	void* result;

	static unsigned int threadCount;
};

}

#endif
