/*****************************************************************************/
/*                                                                           */
/*  Header: iStack.hpp                                                       */
/*                                                                           */
/*  -----------------------------------------------------------------------  */
/*  August 15, 2011                                                          */
/*                                                                           */
/*  Copyright 2010, 2011                                                     */
/*  Tarpini Maximilian, ctrlstudio		                                     */
/*                                                                           */
/*****************************************************************************/


#ifndef __PCLOUD_DATASTRUCT_ISTACK__
#define __PCLOUD_DATASTRUCT_ISTACK__


#include <vector>




/// ////////////////////////////////////////////////////////////////////////////////////////////////
/// iStack /////////////////////////////////////////////////////////////////////////////////////////
template<class iStackParamsT>
class iStack 
{
private:

	struct node
	{
		node* prev;
		iStackParamsT val;
	};

	/////////////////////
	node*			sp;		// stack pointer
	unsigned short	n;		// number of elements


public:

	//! Constructor
	//Creates an empty stack
	__CUDA_HOST_AND_DEVICE__
	INLINE iStack(): sp(NULL), n(0) {}



	//! push_back
	__CUDA_HOST_AND_DEVICE__
	INLINE void push_back(const iStackParamsT& val)
	{
		node* tmp	= new node;

		tmp->val		= val;
		tmp->prev	= sp;
		sp			= tmp;

		++n;
	}

	//! pop
	__CUDA_HOST_AND_DEVICE__
	INLINE iStackParamsT pop(void)
	{
		iStackParamsT val = sp->val;

		node* tmp = sp;
		sp = sp->prev;

		delete tmp;

		--n;
		return val;
	}

	//! clear
	__CUDA_HOST_AND_DEVICE__
	INLINE void clear(void)
	{
		for (node* q = sp; q != NULL; q = sp)        // or sp != NULL
		{
			sp = sp->prev;
			delete q;
		}
		n=0;
	}

	//! full
	__CUDA_HOST_AND_DEVICE__
	INLINE bool full(void)
	{
		node* test= new node;

		if (test==NULL)
			return true;

		delete test;
		return false;
	}

	//! size
	__CUDA_HOST_AND_DEVICE__
	INLINE unsigned short size(void) { return n; }

	//! size
	__CUDA_HOST_AND_DEVICE__
	INLINE unsigned int bsize(void) { return sizeof(node)*n; }

	//! empty
	__CUDA_HOST_AND_DEVICE__
	INLINE bool empty(void) { return sp == NULL; }
};




/// ////////////////////////////////////////////////////////////////////////////////////////////////
/// iStackX ////////////////////////////////////////////////////////////////////////////////////////
#define STACK_N_NODES 64
template<class iStackParamsT>
class iStackX 
{
private:

	/////////////////////
	iStackParamsT *	_stack;		// stack
	iUint			_c_node;	// current element
	iUint			_maxnodes;	// max avail stack slots
public:

	//Constructor
	INLINE iStackX( void ):_maxnodes(STACK_N_NODES)
	{
		_c_node	= 0;
		_stack = (iStackParamsT*) malloc(sizeof(iStackParamsT) * _maxnodes);
	}
	INLINE iStackX( int nslots ):_maxnodes(nslots)
	{
		_c_node	= 0;
		_stack = (iStackParamsT*) malloc(sizeof(iStackParamsT) * _maxnodes);
	}

	//Push back
	INLINE void push_back(const iStackParamsT& val)
	{
		_stack[_c_node]	= val;
		++_c_node;
	}

	//Pop
	INLINE iStackParamsT pop(void)
	{   
		--_c_node;
		return _stack[_c_node];
	}
		
	//Clear
	INLINE void clear(void)
	{
		_c_node=0;
		free(_stack);
	}
		
	//Full
	INLINE bool full(void)
	{
		if (_c_node >= _maxnodes)
		return true;
		return false;
	}
		
	//Size
	INLINE unsigned short size(void) { return _c_node; }

	//Empty
	INLINE bool empty(void) { return _c_node == 0; }
};


#endif
