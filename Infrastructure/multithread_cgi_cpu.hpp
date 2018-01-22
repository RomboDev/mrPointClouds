/*
 * Copyright 2010-2016 by RomboStudios
 * All rights reserved.
 ******************************************************************************

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the software's owners nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 * Created:	Augutst 15, 2011
 */

#ifndef __PCLOUD_MULTITHREAD_CGI_CPU__
#define __PCLOUD_MULTITHREAD_CGI_CPU__


#include <windows.h>						// for HANDLE
#include <process.h>						// for _beginthread()



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Threading class used for CGI parallel computing ///////////////////////////////////////////////////////
template< class CGImpl >
class Thread_CG_Effects_Wrapper
{
	typedef typename CGImpl::PListType			PListType;
	typedef typename CGImpl::iOctreeNodeType	iOctreeNodeType;
	typedef typename CGImpl::iCache				iCache;

	typedef typename CGImpl CGImplT;

	class Thread_CGI_Effects : public CGImpl
	{

	public:
		string	threadName;
		int		threadNumber;

		// constructor
		Thread_CGI_Effects	(	
								int startValue, int endValue, int frequency,
								PListType		* pointlist,
								iOctreeNodeType	* root,
								iCache			* cache 
							)
		: mPointlist(pointlist), mRoot(root), mCache(cache)
		{
			mChunk.startValue	= startValue;
			mChunk.endValue		= endValue;
			mChunk.frequency	= frequency;
		}

		// dummy operator for MVS warning ........
		Thread_CGI_Effects& operator=(const Thread_CGI_Effects& tmp);

		// destructor
		~Thread_CGI_Effects(){}



		// dummy thread entry-point-function /////////////////////////////////////////////////////////////////
		static unsigned __stdcall Thread_CGI_EffectsStaticEntryPoint(void * pThis)
		{
			Thread_CGI_Effects * pthX = (Thread_CGI_Effects*)pThis;	// the tricky cast
			pthX->Thread_CGI_EffectsEntryPoint();					// now call the real entry-point-function

			return 1;												// the thread exit code
		}

		// real entry-point-function /////////////////////////////////////////////////////////////////////////
		void Thread_CGI_EffectsEntryPoint()
		{
			doThread_CG_EffectsPass(&mChunk,						//inherited from CGImpl<>
									mPointlist,
									mRoot,
									mCache,
									false );

			if(mCache->verbose==3) PRINTME( "PHEN 0.3  info : --- thread<%i>, finishing..\n", threadNumber );
		}


		private:
			PListType			* mPointlist;
			iOctreeNodeType		* mRoot;
			iCache				* mCache;
			chunk_idx			mChunk;

	};	// end thread class //////////////////////////////////////////////////////////////////////////////////


	iUint GetChunkSize( const int tpoints, iCache * cache, chunk_idx& chunk)
	{
		int NumTotNodes = tpoints;

		const int TotNumOfThreads = cache->nthreads;
		const int NumOfSubThreads = TotNumOfThreads-1; // ie. minus the main thread

		int ArrayChunkSize = NumTotNodes / TotNumOfThreads;
		int ArrayReminder  = NumTotNodes % TotNumOfThreads;
		if(cache->verbose>=3)
		PRINTME("NumTotNodes:%i, NumTotNodes:%i, ArrayChunkSize:%i, ArrayReminder:%i",
		NumTotNodes, TotNumOfThreads, ArrayChunkSize, ArrayReminder	);

		//main thread chunk
		chunk.startValue	= ArrayChunkSize * (NumOfSubThreads);
		chunk.endValue		= (ArrayChunkSize*(TotNumOfThreads))+ArrayReminder;
		chunk.frequency		= (ArrayChunkSize+ArrayReminder)/(16/(TotNumOfThreads));
		
		chunk.subt_arraychunk = ArrayChunkSize;

		return ArrayChunkSize;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////
	HANDLE				*hThreads;		// handles
	Thread_CGI_Effects	**oThreads;		// threads
	int					NumOfThreads;	// nthreads
	int					verbose;
	chunk_idx			chunk;

public:

	Thread_CG_Effects_Wrapper(){}
	//~Thread_CG_Effects_Wrapper(){ CleanThread_CGI_Effects(); }


	//////////////////////////////////////////////////////////////////////////////////////////////////////
	bool StartMultiThreadComputing(	PListType 			* pointlist,
									iOctreeNodeType		* octRoot,
									iCache				* cache  )
	{	
		NumOfThreads = cache->nthreads-1;
		verbose = cache->verbose;

		int ntotalpoints = pointlist->GetTotalNodes();

		GetChunkSize(ntotalpoints, cache, chunk);
		const int NumOfSubThreads = cache->nthreads-1;
	
		hThreads = (HANDLE*)malloc(sizeof(HANDLE) * NumOfSubThreads);
		oThreads = (Thread_CGI_Effects**)malloc(sizeof(Thread_CGI_Effects) * NumOfSubThreads);

		InitThread_CGI_Effects( chunk.subt_arraychunk, pointlist, octRoot, cache );
		StartThread_CGI_Effects();			// start sub threads


		bool ABORT_CALLED = false;			// start main thread	
		ABORT_CALLED = Thread_CGI_Effects::doThread_CG_EffectsPass( &chunk, pointlist, octRoot, cache, true );

		if(ABORT_CALLED)					// force threads termination
		{
			CleanThread_CGI_Effects();		// del threads

			pointlist->FreeStorage();		// clear data
			delete octRoot;

			return ABORT_CALLED;			// return false if an user abort has been trapped ////////////

		}else
		{
			if( !CGImpl::HasMultiPass )
			ReleaseThread_CGI_Effects();	// free thread stuff
		}
		return false;						// eg. all good, no abort called
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void InitThread_CGI_Effects(int					ArrayChunkSize,
	 
								PListType 			* pointlist,
								iOctreeNodeType		* octRoot,
								iCache				* cache	)
	{

		DWORD dwExitCode;
		for (int t=0; t<cache->nthreads-1; t++)
		{
			// thread
			oThreads[t] = new Thread_CGI_Effects(	ArrayChunkSize*t, ArrayChunkSize*(t+1), 
													ArrayChunkSize/(16/(cache->nthreads)), 
													pointlist, octRoot, 
													cache	);			// thread
			// thread handle
			unsigned  uiThreadID;
			hThreads[t] = (HANDLE)_beginthreadex(	NULL,				// security
													0,					// stack size
													Thread_CGI_Effects::Thread_CGI_EffectsStaticEntryPoint,
													oThreads[t],		// arg list
													CREATE_SUSPENDED,	// so we can later call ResumeThread()
													&uiThreadID	);
			if ( hThreads[t] == 0 ) 
			ERRORME("--- <%i> failure !\n", t);

			GetExitCodeThread( hThreads[t], &dwExitCode );				// should be STILL_ACTIVE = 0x00000103 = 259
			if(cache->verbose>=3) PRINTME( "--- thread<%i>, initialized (with code = %lu)\n", t, dwExitCode );

			oThreads[t]->threadName = "t";
			oThreads[t]->threadNumber = t;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void StartThread_CGI_Effects()
	{
		for (int t=0; t<NumOfThreads; t++)
		ResumeThread( hThreads[t] );   // tx->Start()
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void TerminateThread_CGI_Effects()
	{
		DWORD	dwExitCode= 0xffffffff;
		for (int t=0; t<NumOfThreads; t++)
		{
			TerminateThread(hThreads[t] , dwExitCode);
			if(verbose>=3)
			PRINTME( "--- thread<%i>, terminated ! (with code %lu)\n", t, dwExitCode );

			// close threads
			CloseHandle( hThreads[t]  );

			// delete threads
			delete oThreads[t];
			oThreads[t] = NULL;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void WaitThread_CGI_Effects( bool waitThenClose = false )
	{
		DWORD	dwExitCode= 0xffffffff;
		for (int t=0; t<NumOfThreads; t++)
		{	
			WaitForSingleObject( hThreads[t] , INFINITE );	//wait till they have finished

			GetExitCodeThread( hThreads[t] , &dwExitCode );
			if(verbose>=3)
			PRINTME( "--- thread<%i>, finished (with code %lu)\n", t, dwExitCode );

			if(waitThenClose)
			CloseHandle( hThreads[t]  );
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void ReleaseThread_CGI_Effects()
	{
		DWORD	dwExitCode= 0xffffffff;
		for (int t=0; t<NumOfThreads; t++)
		{	
			WaitForSingleObject( hThreads[t] , INFINITE );	//wait till they have finished

			GetExitCodeThread( hThreads[t] , &dwExitCode );
			if(verbose>=3)
			PRINTME( "--- thread<%i>, exited (with code %lu)\n", t, dwExitCode );
			
			// close threads
			CloseHandle( hThreads[t]  );

			// delete threads
			delete oThreads[t];
			oThreads[t] = NULL;
		}

		CleanThread_CGI_Effects( true );
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void CleanThread_CGI_Effects( bool alreadyterm = false )
	{
		if(!alreadyterm)
		TerminateThread_CGI_Effects();

		free(hThreads);
		free(*oThreads);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////
	bool ThreadedMultiPass( PListType 			* pointlist,
							iOctreeNodeType		* octRoot,
							iCache				* cache,
							bool				ABORT_CALLED )
	{
		if(cache->npasses>1 && !ABORT_CALLED)
		{
			for(int pass = 1; pass < cache->npasses; pass++)
			{
				WaitThread_CGI_Effects( true );												// first pass have been done
				CGImpl::UpdateAOPass( pointlist, octRoot, 0);								// update with last AO

				if(cache->verbose>=2) PRINTME("Occlusion pass<%d> :", pass+1);
				cache->current_pass = pass;
																							// init threads for new pass
				InitThread_CGI_Effects(	chunk.subt_arraychunk,		
										pointlist, octRoot, cache );

				StartThread_CGI_Effects();													// threads are running

				ABORT_CALLED = 
				Thread_CGI_Effects::doThread_CG_EffectsPass(&chunk,							// main thread too
															pointlist, octRoot,
															cache, true );
				if(ABORT_CALLED)		
				{
					CleanThread_CGI_Effects();												// del threads

					pointlist->FreeStorage();												// clear data
					delete octRoot;

					return ABORT_CALLED;													// return true for user abort
				}																			// break passes if abortion


				if(pass == cache->npasses - 1)												// if last pass ..
				{
					bool converge = true;
					WaitThread_CGI_Effects();												// wait threads to finish
					CGImpl::UpdateAOPass( pointlist, octRoot, 0, converge,
													  cache->verbose);						// then converge occlusion
				}
			}
		}

		ReleaseThread_CGI_Effects();
		return false;																		// eg. all good, no abort called
	}


};	//end thread

#endif __PCLOUD_MULTITHREAD_CGI_CPU__