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

#ifndef __PCLOUD_MULTITHREAD_CPU__
#define __PCLOUD_MULTITHREAD_CPU__


#include <windows.h>						// for HANDLE
#include <process.h>						// for _beginthread()



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Threading class used for CGI parallel computing ///////////////////////////////////////////////////////
template
<
typename DataType, 
class PListType, 
class iOctreeNodeType, 
typename iCache, 
template<typename, class, class, class> class CGImpl
>
class Thread_CG_Effects : public CGImpl< DataType, PListType, iOctreeNodeType, iCache >
{

public:

	typedef CGImpl< DataType, PListType, iOctreeNodeType, iCache > _iMplT;
	typedef iCache iCache;

	string	threadName;
	int		threadNumber;

	// constructor
	Thread_CG_Effects	(	
							int startValue, int endValue, int frequency,
							PListType		* pointlist,
							iOctreeNodeType	* root,
							iCache			const*const  cache 
						)
	: mPointlist(pointlist), mRoot(root), mCache(cache)
	{
		mChunk.startValue = startValue;
		mChunk.endValue = endValue;
		mChunk.frequency = frequency;
	}

	// dummy operator for MVS warning ........
	Thread_CG_Effects& operator=(const Thread_CG_Effects& tmp);

	// destructor
	~Thread_CG_Effects(){}



	// dummy thread entry-point-function /////////////////////////////////////////////////////////////////
	static unsigned __stdcall Thread_CG_EffectsStaticEntryPoint(void * pThis)
	{
		Thread_CG_Effects * pthX = (Thread_CG_Effects*)pThis;	// the tricky cast
		pthX->Thread_CG_EffectsEntryPoint();						// now call the real entry-point-function

		return 1;												// the thread exit code
	}

	// real entry-point-function /////////////////////////////////////////////////////////////////////////
	void Thread_CG_EffectsEntryPoint()
	{
		doThread_CG_EffectsPass(&mChunk,						//inherited from CGImpl<>
								mPointlist,
								mRoot,
								mCache,
								false );

		if(mCache->verbose==3) PRINTME( "PHEN 0.3  info : --- thread<%i>, finishing..\n", threadNumber );
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////
	static
	void InitThread_CG_Effects(	HANDLE					* hThreads,
								Thread_CG_Effects		**oThreads,
								int						ArrayChunkSize,
	 
								PListType 				* pointlist,
								iOctreeNodeType			* octRoot,
								iCache					const*const  cache	)
	{

		DWORD dwExitCode;
		for (int t=0; t<cache->nthreads-1; t++)
		{
			// thread
			oThreads[t] = new Thread_CG_Effects(	ArrayChunkSize*t, ArrayChunkSize*(t+1), 
													ArrayChunkSize/(16/(cache->nthreads)), 
													pointlist, octRoot, 
													cache	);			// thread
			// thread handle
			unsigned  uiThreadID;
			hThreads[t] = (HANDLE)_beginthreadex(	NULL,				// security
													0,					// stack size
													Thread_CG_Effects::Thread_CG_EffectsStaticEntryPoint,
													oThreads[t],		// arg list
													CREATE_SUSPENDED,	// so we can later call ResumeThread()
													&uiThreadID	);
			if ( hThreads[t] == 0 ) 
			ERRORME("--- <%i> failure !\n", t);

			GetExitCodeThread( hThreads[t], &dwExitCode );				// should be STILL_ACTIVE = 0x00000103 = 259
			if(cache->verbose>=3) PRINTME( "--- thread<%i>, initialized (with code = %u)\n", t, dwExitCode );

			oThreads[t]->threadName = "t";
			oThreads[t]->threadNumber = t;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	static
	void StartThread_CG_Effects(HANDLE *hThreads, int NumOfThreads)
	{
		for (int t=0; t<NumOfThreads; t++)
		ResumeThread( hThreads[t] );   // tx->Start()
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	static
	void TerminateThread_CG_Effects(HANDLE				* hThreads,
									Thread_CG_Effects	** oThreads,
									iCache				const * const cache	)
	{
		DWORD	dwExitCode= 0xffffffff;
		for (int t=0; t<cache->nthreads-1; t++)
		{
			TerminateThread(hThreads[t] , dwExitCode);
			if(cache->verbose>=3)
			PRINTME( "--- thread<%i>, terminated ! (with code %u)\n", t, dwExitCode );

			// close threads
			CloseHandle( hThreads[t]  );

			// delete threads
			delete oThreads[t];
			oThreads[t] = NULL;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	static
	void WaitThread_CG_Effects(	HANDLE		*hThreads,
								iCache		const * const cache,
								bool		waitThenClose = false )
	{
		DWORD	dwExitCode= 0xffffffff;
		for (int t=0; t<cache->nthreads-1; t++)
		{	
			WaitForSingleObject( hThreads[t] , INFINITE );	//wait till they have finished

			GetExitCodeThread( hThreads[t] , &dwExitCode );
			if(cache->verbose>=3)
			PRINTME( "--- thread<%i>, finished (with code %u)\n", t, dwExitCode );

			if(waitThenClose)
			CloseHandle( hThreads[t]  );
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	static
	void ReleaseThread_CG_Effects(	HANDLE				*hThreads,
									Thread_CG_Effects	**oThreads,
									iCache				const * const cache	)
	{
		DWORD	dwExitCode= 0xffffffff;
		for (int t=0; t<cache->nthreads-1; t++)
		{	
			WaitForSingleObject( hThreads[t] , INFINITE );	//wait till they have finished

			GetExitCodeThread( hThreads[t] , &dwExitCode );
			if(cache->verbose>=3)
			PRINTME( "--- thread<%i>, exited (with code %u)\n", t, dwExitCode );

			// close threads
			CloseHandle( hThreads[t]  );

			// delete threads
			delete oThreads[t];
			oThreads[t] = NULL;
		}

		CleanThread_CG_Effects( hThreads,oThreads,cache, true );
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	static
	void CleanThread_CG_Effects(	HANDLE				*hThreads,
									Thread_CG_Effects	**oThreads,
									iCache				const * const cache,
									bool				alreadyterm = false)
	{
		if(!alreadyterm)
		TerminateThread_CG_Effects(hThreads, oThreads, cache);

		mi_mem_release(hThreads);
		mi_mem_release(*oThreads);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//static iUint GetChunkSize( const int tpoints, const int nthreads, const int verbose )
	static iUint GetChunkSize( const int tpoints, iCache const * const cache, chunk_idx& chunk)
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

		return ArrayChunkSize;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////
	static	//NOOONE ... 
	bool DoMultiThreadPass( HANDLE				*hThreads,
							Thread_CG_Effects	**oThreads,
							PListType			* _pointlist,
							iOctreeNodeType		* _octroot,
							iCache				const * const cache,
							iUint				const ntotalpoints)
	{
		typedef Thread_CG_Effects
		<	
			float, 
			PListType, 
			iOctreeNodeType,
			iCache,
			CGImpl
		>  
		_oThreadT;

		// Init multithread stuff ////////////////////////////////////////////////////////////////////////////
		chunk_idx	chunk;
		int ArrayChunkSize = GetChunkSize(ntotalpoints, cache, chunk);
		const int NumOfSubThreads = cache->nthreads-1;


		// Create Threads ////////////////////////////////////////////////////////////////////////////////////
		hThreads = (HANDLE*)mi_mem_allocate(sizeof(HANDLE) * NumOfSubThreads);
		oThreads = (_oThreadT**)mi_mem_allocate(sizeof(_oThreadT) * NumOfSubThreads);

		InitThread_CG_Effects(	hThreads, oThreads, ArrayChunkSize, _pointlist, _octroot, cache );
		StartThread_CG_Effects( hThreads, NumOfSubThreads);	// threads are running ///////////////


		// Process data on main thread ///////////////////////////////////////////////////////////////////////
		miBoolean ABORT_CALLED = miFALSE;					//if abort, break here and then terminate threads
		ABORT_CALLED = doThread_CG_EffectsPass( &chunk, _pointlist, _octroot, cache, true );
															// eventually, start main thread too ////////////

		return ABORT_CALLED;
	}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	PListType			* mPointlist;
	iOctreeNodeType		* mRoot;
	iCache				const * mCache;
	chunk_idx			mChunk;

};	//end thread

#endif __PCLOUD_MULTITHREAD_CPU__