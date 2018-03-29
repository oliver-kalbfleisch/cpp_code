/*
    Thread Pool implementation for unix / linux environments
    Copyright (C) 2008 Shobhit Gupta
	
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include "threadpool.h"

using namespace std;

ThreadPool::ThreadPool()
{
	ThreadPool(2);
}

ThreadPool::ThreadPool(int maxThreads)
{
   if (maxThreads < 1)  maxThreads=1;
   
   mutexSync = PTHREAD_MUTEX_INITIALIZER;
   mutexWorkCompletion = PTHREAD_MUTEX_INITIALIZER;
   
   pthread_mutex_lock(&mutexSync);
   this->maxThreads = maxThreads;
   this->queueSize = maxThreads;
   workerQueue = new WorkerThread*[maxThreads];
   topIndex = 0;
   bottomIndex = 0;
   incompleteWork = 0;
   sem_init(&availableWork, 0, 0);
   sem_init(&availableThreads, 0, queueSize);
   pthread_mutex_unlock(&mutexSync);
}

void ThreadPool::initializeThreads()
{
   for(int i = 0; i<maxThreads; ++i)
	{
		pthread_t tempThread;
		pthread_create(&tempThread, NULL, &ThreadPool::threadExecute, (void *) this ); 
		 //threadIdVec[i] = tempThread;
   }

}

ThreadPool::~ThreadPool()
{
}



void ThreadPool::destroyPool(int maxPollSecs = 5)
{
	while( incompleteWork>0 )
	{
	//cout << "Work is still incomplete=" << incompleteWork << endl;
		sleep(maxPollSecs);
	}
	cout << "All Done!! Wow! That was a lot of work!" << endl;
	sem_destroy(&availableWork);
	sem_destroy(&availableThreads);
}


bool ThreadPool::assignWork(WorkerThread *workerThread)
{
	pthread_mutex_lock(&mutexWorkCompletion);
		incompleteWork++;
		//cout << "assignWork...incomapleteWork=" << incompleteWork << endl;
	pthread_mutex_unlock(&mutexWorkCompletion);
    
	sem_wait(&availableThreads);
	
	pthread_mutex_lock(&mutexSync);
		//workerVec[topIndex] = workerThread;
		workerQueue[topIndex] = workerThread;
		if(queueSize !=1 )
			topIndex = (topIndex++) % (queueSize-1);
		sem_post(&availableWork);
	pthread_mutex_unlock(&mutexSync);
	return true;
}

bool ThreadPool::fetchWork(WorkerThread **workerArg)
{
	sem_wait(&availableWork);

	pthread_mutex_lock(&mutexSync);
		WorkerThread * workerThread = workerQueue[bottomIndex];
		*workerArg = workerThread;
		if(queueSize !=1 )
			bottomIndex = (bottomIndex++) % (queueSize-1);
		sem_post(&availableThreads);
	pthread_mutex_unlock(&mutexSync);
    return true;
}

void *ThreadPool::threadExecute(void *param)
{
	WorkerThread *worker = NULL;
	
	while(((ThreadPool *)param)->fetchWork(&worker))
	{
		if(worker)
			worker->executeThis();

		pthread_mutex_lock( &(((ThreadPool *)param)->mutexWorkCompletion) );
			((ThreadPool *)param)->incompleteWork--;
		pthread_mutex_unlock( &(((ThreadPool *)param)->mutexWorkCompletion) );
	}
	return 0;
}
