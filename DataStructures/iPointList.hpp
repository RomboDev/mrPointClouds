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


#ifndef __PCLOUD_DATASTRUCT_IPOINTLIST__
#define __PCLOUD_DATASTRUCT_IPOINTLIST__

#include <string.h>
//#include <Partio.h> .. not used .. so for here TransferMISurfelDataToPTC below and pcloud_Bake.cpp line 270

namespace cpc
{


/// Check what's used below !! ////////////////////////////////////////////////////////////////////////////////////////

enum iPListNodeAllocatorMode
{
	MINIMAL		= DATA_SLOT_MINIMAL,
    AO			= DATA_NSLOTS_AO,
    AO_TRIVIS	= DATA_NSLOTS_AO_TRIVIS,
    SSS			= DATA_NSLOTS_SSS
};


////////////////////////////////////////////////////////////////////////////////////////////////////
// Load float vectors to XMM registers - Template unrolling ////////////////////////////////////////
template< typename DataType=float, unsigned START=0, unsigned I=0>
struct LoadXMMr
{
	static inline void eval( __m128* p, DataType* q )
	{
		p[START] = _mm_load_ps( q + I*4 );
		LoadXMMr< DataType, START+1, I+1 >::eval( p,q );
	}
};
// Partial Template Specialization
template <typename DataType, unsigned START> 
struct LoadXMMr<DataType, START, 3>
{
	static inline void eval( __m128* p, DataType* q )
	{
		p[3] = _mm_load_ps( q + 12);
	}
};

////////////////////////////////////////////////////////////////////////////////////////////////////
template< typename DataType, unsigned NSlots, unsigned I>
struct StoreXMM
{
	static INLINE void eval( DataType* p, DataType* q )
	{
		__m128 const& XMM = _mm_load_ps( q+ I*4 );
		_mm_store_ps( p+ I*4, XMM );
		StoreXMM< DataType, NSlots, I-1 >::eval( p,q );
	}
};
// Partial Template Specialization
template <typename DataType, unsigned NSlots> 
struct StoreXMM<DataType, NSlots, 0>
{
	static INLINE void eval( DataType* p, DataType* q )
	{
		__m128 const& XMM = _mm_load_ps( q );
		_mm_store_ps( p, XMM );
	}
};


////////////////////////////////////////////////////////////////////////////////////////////////////
template<class DSTORAGE>
class iPListDataProjection
{
public:
	iPListDataProjection(){}
	~iPListDataProjection(){}
};


/// Node view /////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int temp_n_slots = 16;

template<typename DataType, unsigned Size>
class iPListNode
{

	enum{ arXMM = Size/2, arXMM_Div2 = arXMM/2 };

public:
	INLINE iPListNode( DataType* dataptr ):dwrapper(dataptr){;}
	INLINE ~iPListNode(){}

	INLINE		 float& operator[](const iUint i)		{ return dwrapper[i]; };
	INLINE const float& operator[](const iUint i) const	{ return dwrapper[i]; };

	INLINE operator			DataType* ( void )			{ return dwrapper; }
	INLINE operator const	DataType* ( void ) const	{ return dwrapper; }

	////////////////////////////////////////////////////////////////////////////////////////////////

	// Swap
	INLINE void swap( iPListNode<float, Size> & q)
	{
		__m128 XMM[ arXMM ];

		//#pragma unroll (arXMM_Div2)
		//for(int ii=0; ii < arXMM_Div2; ii++)
		//XMM[ii] = _mm_load_ps( dwrapper+ (ii*4) );
		LoadXMMr<>::eval( XMM, dwrapper);

		//#pragma unroll (arXMM_Div2)
		//for(int jj=0; jj < arXMM_Div2; jj++)
		//XMM[jj+4] = _mm_load_ps( q + (jj*4) );
		LoadXMMr<float, 4, 0>::eval( XMM, q);

		#pragma unroll (arXMM_Div2)
		for(int ii=0; ii < arXMM_Div2; ii++)
		_mm_store_ps( dwrapper+ ii*4, XMM[ii+4] );

		#pragma unroll (arXMM_Div2)
		for(int jj=0; jj < arXMM_Div2; jj++)
		_mm_store_ps( q+ jj*4   , XMM[jj]);
	}

private:
	DataType* dwrapper;
};

// Standalone swap for std::sort
INLINE void swap( iPListNode<float,temp_n_slots>& nFirst,	iPListNode<float,temp_n_slots>& nSecond)
{
	nFirst.swap(nSecond);
}



/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iPListDataStorage /////////////////////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename DataType, unsigned NodeSize>
class iPListDataStorage_Simple
{
public:
	typedef DataType _DTYPE;

	//////////////////////////////////////
	INLINE iPListDataStorage_Simple():_totalpoints(0){};
	INLINE ~iPListDataStorage_Simple(){}

	INLINE bool					IsStorageConsistent	( void )					{ return _pointlist!=NULL; }
	INLINE const DataType**		GetStorageArray		( void )					{ return &_pointlist[0];}
	INLINE void					SetStorageArray		( const DataType** aplist )	{ _pointlist = aplist;}

	INLINE iUint				GetTotalNodes		( void )					{ return _totalpoints; }
	INLINE void					SetTotalNodes		( iUint _ntpoints )			{ _totalpoints=_ntpoints; }

	INLINE DataType*			GetNode				( iUint _id )				{ return _pointlist[_id]; }
	INLINE DataType&			GetNodeRef			( iUint _id )				{ return *_pointlist[_id]; }
	INLINE const DataType*		GetNodePtr			( iUint _id )				{ return _pointlist[_id]; }
	INLINE void					SetNode		( DataType* _node, const int _id )	{ _pointlist[_id] = _node; }

private:
	const DataType**	_pointlist;
	iUint				_totalpoints;
};


template<typename DataType, unsigned NodeSize>
class iPListDataStorage_STDVector
{
public:
	typedef DataType _DTYPE;

	///////////////////////////////////////////////
	INLINE iPListDataStorage_STDVector():_totalpoints(0){}
	INLINE ~iPListDataStorage_STDVector(){}

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE bool			AllocateStorage( const iUint _ntotalpoints=_totalpoints, bool realloc=false )
							{
								if(_totalpoints==0 || realloc)
								{
									if(realloc) FreeStorage();
									_totalpoints = _ntotalpoints;
								}else
								{
									_totalpoints += _ntotalpoints;
								}
								
								_vpointlist.reserve(_totalpoints);///TODO:BLOCKMEM:WITH:RESIZE:(TOTPOINTS*DATASIZE):NO:NEED:TO:NEW:NODE:BELOW:??!
								return true;
							}
	INLINE void			FreeStorage( void )
							{

								for( iUint i=0; i<_totalpoints; i++ )
								MEMFREE( _vpointlist[i] );

								_vpointlist.clear();
								_vpointlist.resize(0);

								_totalpoints = 0;
							}

	INLINE DataType*	AllocateNewNode( void )
							{
								DataType* datanode = (DataType*) MEMALLOC( DataType, NodeSize );
								if(datanode == NULL){ std::cout << "\nNodeAllocation failure .. aborting." << endl; return NULL; }
								return datanode;
							}

	INLINE bool						IsStorageConsistent( void )	{ return !_vpointlist.empty(); }

	INLINE vector<DataType*>*		GetStorageVector( void )	{ return &(_vpointlist); }
	INLINE void						SetStorageVector( vector<DataType*> vplist )	{ _vpointlist=vplist; }

	INLINE const DataType**			GetStorageArray( void )		{ return (const DataType**)&_vpointlist[0]; }
	INLINE void						SetStorageArray( DataType** aplist ){ _vpointlist = aplist; }

	INLINE iUint					GetTotalNodes( void )		{ return _totalpoints; }
	INLINE void						SetTotalNodes( iUint _ntpoints ){ _totalpoints=_ntpoints; }

	INLINE iPListNode<DataType,4>	GetNode( iUint _id )		{ return _vpointlist[_id]; }
	INLINE DataType*				GetNodePtr( iUint _id )		{ return _vpointlist[_id]; }
	INLINE DataType&				GetNodeRef( iUint _id )		{ return *_vpointlist[_id]; }

	INLINE void						AddNode( DataType* _node ) { _vpointlist.push_back( _node ); }

private:
	vector<DataType*>	_vpointlist;
	iUint				_totalpoints;
};


/// Memory pool ///////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename DataType, unsigned NodeSize>
class iPListDataStorage_MemPool
{

public:
	typedef DataType _DTYPE;

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE iPListDataStorage_MemPool():_totalpoints(0), currentNode(0){};
	INLINE ~iPListDataStorage_MemPool(){}

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE bool			AllocateStorage( const iUint _ntotalpoints = _totalpoints, bool realloc=false )
							{
								if(_totalpoints==0 || realloc)
								{
									if(realloc) FreeStorage();

									_totalpoints = _ntotalpoints;

									DataType* malloced = (DataType*) MEMALLOC( DataType, _ntotalpoints * NodeSize );
									if(malloced!=NULL)
									{
										_pointlist = malloced;
										return true;
									}else
									{
										_totalpoints=0;
										ERRORME("--- memory allocation failure !");
										return false;
									}
								}else
								{
									_totalpoints += _ntotalpoints;

									DataType*	extend_plist = 
										(DataType*) MEMREALLOC(_pointlist, DataType, _totalpoints * NodeSize);

									if(extend_plist != NULL){
										_pointlist = extend_plist;
										return true;
									}
									else{
										WARNME("--- memory reallocation failure !");
										_totalpoints -= _ntotalpoints;
										return false;
									}
								}
							}
	INLINE void			FreeStorage( void ){ MEMFREE( _pointlist ); _totalpoints==0; }

	INLINE DataType*	AllocateNewNode( void )
							{
								DataType* datanode = &(_pointlist[currentNode * NodeSize]);
								currentNode++;
								return datanode;
							}

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE bool			IsStorageConsistent( void )			{ return _pointlist!=NULL; }
	INLINE DataType*	GetStorageArray( void )				{ return &_pointlist[0]; }
	INLINE vector<DataType>*GetStorageVector( void )		{ return vector<DataType>(_pointlist[0], _pointlist[_totalpoints]);}

	INLINE iUint		GetTotalNodes( void )				{ return _totalpoints; }
	INLINE void			SetTotalNodes( iUint _ntpoints )	{ _totalpoints=_ntpoints; }

	INLINE DataType&	GetNodeRef( iUint _id )				{ return _pointlist[_id * NodeSize]; }
	INLINE DataType*	GetNodePtr( iUint _id )				{ return &(_pointlist[_id * NodeSize]); }
	INLINE iPListNode<DataType,4>	GetNode( iUint _id )	{ return &(_pointlist[_id * NodeSize]); }
	
	INLINE void			AddNode( DataType* _node ){(void)_node;}	//nothing todo

private:
	DataType*	_pointlist;
	iUint		_totalpoints;
	iUint		currentNode;

};


/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iPListDataAllocator ///////////////////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template
< 
typename DataType,
unsigned NodeSize,
template<typename,unsigned> class iDataStorage
>
class iPListDataAllocatorBase : public iDataStorage<DataType, NodeSize>
{
public:
	typedef iDataStorage<DataType, NodeSize>	_DataStorage;

	/////////////////////////////////////////////////////////////
	INLINE iPListDataAllocatorBase():_DataStorage(){}
	INLINE ~iPListDataAllocatorBase(){}


	template<unsigned NODEARRAYPOS> 
	INLINE DataType& GetNodeVectorRef( iUint NodeID ){ return _DataStorage::GetNode(NodeID)[NODEARRAYPOS]; }

	template<unsigned NODEARRAYPOS> 
	INLINE iPoint<DataType,3> GetNodeVectorAt( const iUint& NodeID )
					{ return iPoint<DataType,3>( &(_DataStorage::GetNode( NodeID )[ NODEARRAYPOS ]) ); }
	template<unsigned NODEARRAYPOS> 
	INLINE iPoint<DataType,3> GetNodeVectorAt( DataType* Node )
					{ return iPoint<DataType,3>( &( Node[ NODEARRAYPOS ]) ); }
	template<unsigned NODEARRAYPOS> 
	INLINE iPoint<DataType,3> GetNodeVectorAt( const DataType* Node )
					{ return iPoint<DataType,3>( &( Node[ NODEARRAYPOS ]) ); }
	template<unsigned NODEARRAYPOS> 
	INLINE DataType	GetNodeScalarAt( const iUint& NodeID )
					{ return _DataStorage::GetNode( NodeID )[ NODEARRAYPOS ]; }
	template<unsigned NODEARRAYPOS> 
	INLINE DataType	GetNodeScalarAt( DataType* Node )
					{ return Node[ NODEARRAYPOS ]; }
	template<unsigned NODEARRAYPOS> 
	INLINE DataType	GetNodeScalarAt( const DataType* Node )
					{ return Node[ NODEARRAYPOS ]; }

	template<unsigned NODEARRAYPOS> 
	INLINE void		SetNodeScalarAt( const iUint& NodeID, const DataType& val )
					{ _DataStorage::GetNode( NodeID )[NODEARRAYPOS] = val; }
	template<unsigned NODEARRAYPOS>	
	INLINE void		SetNodeScalarAt( DataType* Node, const DataType& val )
					{ Node[NODEARRAYPOS] = val; }
};


/// Float allocator ///////////////////////////////////////////////////////////////////////////////////////////////////
template
< 
typename DataType,
unsigned NodeSize,
template<typename,unsigned> class iDataStorage
>
class iPListDataAllocator : public iPListDataAllocatorBase< DataType,NodeSize,iDataStorage >
{

	enum{arrend =NodeSize-1};
	DataType holdswap[NodeSize];

	////////////////////////////////////////////////////////////////////////////////////////////////
	template< typename DType=DataType, unsigned SIZE=arrend, unsigned I=0 >
	struct StoreNodeBlock
	{
		static inline void eval( DType* p, DType* q )
		{
			p[I] = q[I];
			StoreNodeBlock< DType, SIZE, I+1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DType, unsigned SIZE> 
	struct StoreNodeBlock<DType, SIZE, SIZE>
	{
		static inline void eval( DType* p, DType* q )
		{
			p[SIZE] = q[SIZE];
		}
	};

	////////////////////////////////////////////////////////////////////////////////////////////////
	template< typename DTYPE=DataType, unsigned ISLOT=0, unsigned I=0 > 
	struct CopyFieldBlock
	{
		static inline void eval( DTYPE* p, DTYPE* q )
		{
			p[ISLOT]  = q[I];
			CopyFieldBlock< DTYPE, ISLOT+1, I+1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DTYPE, unsigned ISLOT> 
	struct CopyFieldBlock<DTYPE, ISLOT, 2>
	{
		static inline void eval( DTYPE* p, DTYPE* q )
		{
			p[ISLOT] = q[2];
		}
	};


public:

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE iPListDataAllocator(){}
	INLINE ~iPListDataAllocator(){}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Copy surfel to node
	INLINE void NodeCopy( DataType *p, DataType *q )
	{
		StoreNodeBlock<DataType,NodeSize-DATA_RESULT_SLOT-1>::eval( p,q );
	};

	// Fill plist node with defaults
	INLINE void NodeFillSlots( DataType *p )
	{
		// last 3 slots (d3)
		float aodefault[4] = {1.f,1.f,.6f};
		CopyFieldBlock<DataType, NodeSize-DATA_RESULT_SLOT, 0>::eval(p, aodefault);
	};
	// Copy field data
	INLINE void StoreFieldData( DataType * plistnode, const iPoint<DataType, 3>& fielddata )
	{
		CopyFieldBlock<>::eval( plistnode, (DataType*)fielddata.begin() );
	}
	INLINE void StoreFieldData( DataType * plistnode, const DataType fielddata )
	{
		plistnode[0] = fielddata;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE iUint GetNodeSize( void ){ return NodeSize; }

	template<unsigned NODEARRAYPOS> 
	INLINE void		SetNodeVectorAt( const iUint& NodeID, const iPoint<DataType, 3>& Pos )
					{ 
						CopyFieldBlock<DataType>::eval( &(_DataStorage::GetNode( NodeID )[NODEARRAYPOS]), (DataType*)Pos.begin() ); 
					}

	template<unsigned NODEARRAYPOS>	
	INLINE void		SetNodeVectorAt( DataType* Node, const iPoint<DataType, 3>& Pos )
					{ 
						CopyFieldBlock<DataType, NODEARRAYPOS>::eval( Node, (DataType*)Pos.begin() ); 
					}

	INLINE void		SwapNodes(iUint i, iUint j)
					{	
						StoreNodeBlock<>::eval(holdswap, GetNode(i));
						StoreNodeBlock<>::eval(GetNode(i), GetNode(j));
						StoreNodeBlock<>::eval(GetNode(j), holdswap);
					}
};


/// SSE allocator /////////////////////////////////////////////////////////////////////////////////////////////////////
template
< 
typename DataType,
unsigned NodeSize,
template<typename,unsigned> class iDataStorage
>
class iPListDataAllocator_SSE : public iPListDataAllocatorBase< DataType,NodeSize,iDataStorage >
{

	enum{	arXMM =NodeSize/2,
			arXMM_Div2 =arXMM/2,
			beginResultSlot =NodeSize-DATA_RESULT_SLOT,
			iSurfleXMMSlots =((NodeSize)/4)-1
		};

	////////////////////////////////////////////////////////////////////////////////////////////////
	template< typename DType=DataType, unsigned NSlots=iSurfleXMMSlots, unsigned I=0>
	struct StoreNodeBlock
	{
		static INLINE void eval( DType* p, DType* q )
		{
			__m128 const& XMM = _mm_load_ps( q+ I*4 );
			_mm_store_ps( p+ I*4, XMM );
			StoreNodeBlock< DType, NSlots, I+1 >::eval( p,q );
		}
	};
	// Partial Template Specialization
	template <typename DType, unsigned NSlots> 
	struct StoreNodeBlock<DType, NSlots, NSlots>
	{
		static INLINE void eval( DType* p, DataType* q )
		{
			(void)q;
			_mm_store_ps( p+ NSlots*4, M128_ONE );	//fill result slot with 1s

			//__m128 const& XMM = _mm_load_ps( q+ NSlots*4 );
			//_mm_store_ps( p+ NSlots*4, XMM );
		}
	};


public:

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE iPListDataAllocator_SSE(){}
	INLINE ~iPListDataAllocator_SSE(){}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Copy surfel to node
	INLINE void NodeCopy( DataType *p, DataType *q )
	{
		StoreNodeBlock<>::eval( p, q );
	};
	// Fill plist node with defaults
	INLINE void NodeFillSlots( DataType *p )
	{
		// last 3 slots (d3)
		_mm_store_ps( p+ beginResultSlot, M128_ONE );
	};
	// Copy field data
	INLINE void StoreFieldData( DataType * plistnode, const iPoint<DataType, 3>& fielddata )
	{
		_mm_store_ps( plistnode, fielddata );
	}
	INLINE void StoreFieldData( DataType * plistnode, const DataType fielddata )
	{
		_mm_store_ps( plistnode, _mm_setr_ps(fielddata, 1.f,1.f,1.f) );
	}

	////////////////////////////////////////////////////////////////////////////////////////////////
	INLINE iUint GetNodeSize( void ){ return NodeSize; }

	template<unsigned NODEARRAYPOS> 
	INLINE void		SetNodeVectorAt( const iUint& NodeID, const iPoint<DataType, 3>& Pos )
					{ _mm_store_ps(&(_DataStorage::GetNode( NodeID )[NODEARRAYPOS]), Pos ); }
	template<unsigned NODEARRAYPOS>	
	INLINE void		SetNodeVectorAt( DataType* Node, const iPoint<DataType, 3>& Pos )
					{ _mm_store_ps(&(Node[ NODEARRAYPOS ]), Pos ); }

	INLINE void		SwapNodes(iUint i, iUint j)
					{	
						__m128 XMM[ arXMM ];

						#pragma unroll (arXMM_Div2)
						for(int ii=0; ii < arXMM_Div2; ii++)
						XMM[ii] = _mm_load_ps(_DataStorage::GetNodePtr(i)+ (ii*4));

						#pragma unroll (arXMM_Div2)
						for(int jj=0; jj < arXMM_Div2; jj++)
						XMM[jj +arXMM_Div2] = _mm_load_ps(_DataStorage::GetNodePtr(j)+ (jj*4));

						#pragma unroll (arXMM_Div2)
						for(int ii=0; ii < arXMM_Div2; ii++)
						_mm_store_ps(_DataStorage::GetNodePtr(i)+ ii*4, XMM[ii +arXMM_Div2]);

						#pragma unroll (arXMM_Div2)
						for(int jj=0; jj < arXMM_Div2; jj++)
						_mm_store_ps(_DataStorage::GetNodePtr(j)+ jj*4, XMM[jj]);
					}
};



/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iPListDataLayout //////////////////////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class iPListDataLayout
{
public:
	struct DataFlag
	{
		string    	name;
		int	    	type;
		int			offset;

		DataFlag():offset(0){}
	};

	iPListDataLayout(){}

	INLINE void AddGlobalField(const DataFlag& iAttr){ global_fields.push_back(iAttr); }
	INLINE void AddLocalField(const DataFlag& iAttr){ local_fields.push_back(iAttr); }

	INLINE const vector<DataFlag>& GetGlobalFields(void){ return global_fields; }
	INLINE const vector<DataFlag>& GetLocalFields(void){ return local_fields; }

	inline iUint GetFieldType(const char* chartype)
	{
		iUint ret_value = 0;
		if		(strcmp ( chartype , "i\0" ) == 0) ret_value=1; 	//int
		else if (strcmp ( chartype , "s\0" ) == 0) ret_value=2; 	//scalar
		else if (strcmp ( chartype , "v\0" ) == 0) ret_value=3; 	//vector
		else if	(strcmp ( chartype , "c\0" ) == 0) ret_value=4; 	//vector
		else									   ret_value=0;		//default::color

		return ret_value;
	}
	inline void WriteField( FILE* ofp, const DataFlag& field, bool isglobal=0)
	{
		if ( field.type == 1 ) 		fprintf ( ofp , "i" );
		else if ( field.type == 2 ) fprintf ( ofp , "s" );
		else if ( field.type == 3 ) fprintf ( ofp , "v" );
		else if	( field.type == 4 ) fprintf ( ofp , "c" );
		else						fprintf ( ofp , "$" );

		// the flag that says it's a global field
		if(isglobal) fprintf ( ofp , "g" );

		// field name .. we write also the quotes around the name
		string tname( "\"" + field.name + "\"" );
		fprintf ( ofp , "%s" , tname.c_str() );
	}
	inline iUint GetFieldDataOffset(const iUint typ)
	{
		#if __SSE2_ENHANCED__
		if(typ) return DATA_RESULT_SLOT;
		else	return 1;
		#else
		if(typ==1)		return 1;
		else if(typ==2) return 1;
		else if(typ==3) return 3;
		else return 3;	//rgba
		#endif
	}
	inline int GetTotalFields( const bool gfields=false )
	{
		if(gfields)	return (int) global_fields.size(); 
		else		return (int) local_fields.size(); 
	}

	void PrintDataLayout(void)
	{
		PRINTME("\n\nPointList::Layout::LocalDataFlags<%i> : \n",(int)local_fields.size());
		for(iUint i=0; i<local_fields.size(); i++)
		{  
			string s = local_fields[i].name;
			s.erase( remove( s.begin(), s.end(), '\"' ), s.end() );
			PRINTME ("--- field type <%i>, with name < %s > and offset < %i > \n" ,
					local_fields[i].type, s.c_str() /*local_fields[i].name.c_str()*/, local_fields[i].offset );
		}
	}
	
	void ClearDataLayout(void)	//TODO::AUTOMATICALLY:CLEAR:DATA:LAYOUT:!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	{
		global_fields.clear();
		local_fields.clear();
	}
private:
	vector<DataFlag>  global_fields;
	vector<DataFlag>  local_fields;
};



/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iPointList - Specialized container class //////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template
<
typename DataType	= float,
unsigned NodeSize	= DATA_SLOT_MINIMAL,

template<typename,unsigned>										class iDataStorage		= iPListDataStorage_MemPool,
template<typename,unsigned, template<typename,unsigned> class >	class iNodeAllocator	= iPListDataAllocator_SSE,
template<typename>												class iDataProjector	= iPListDataProjection
>
class iPointList
	:	public	iNodeAllocator<DataType, NodeSize, iDataStorage	>
	,	public	iPListDataLayout
{

public:
	typedef DataType											_iDataType;
	typedef iDataStorage<DataType,NodeSize>						_iDataStorage;
	typedef iNodeAllocator<DataType, NodeSize, iDataStorage>	_iNodeAllocator;
	typedef iDataProjector<_iDataStorage>						_iDataProjection;
	typedef iPListDataLayout::DataFlag							_iDataFlag;

	enum{ NODESIZE = NodeSize, NODERESULTSLOT = NodeSize-DATA_RESULT_SLOT };


	//////////////////////////////////////////////////////////
	iPointList( void ):_iNodeAllocator(), iPListDataLayout(){}
	~iPointList(){}

	////////////////////////////////////////////////////////////////////////////////////////////////
	// node vector handling
	INLINE DataType* operator ()(const iUint i, const iUint j){ return &(_iDataStorage::GetNodePtr(i)[j] ); };

	// utilities
	void PrintDataInfo( const int verbose )
	{ 
		if(verbose>=2) PRINTME("--- total nodes<%u>\n", _iDataStorage::GetTotalNodes());
		if(verbose>=3) {
			int nodedatasize = sizeof(DataType)*NodeSize;
			#if		__SSE2_ENHANCED__
			nodedatasize = ((nodedatasize/ALIGNPAD)+1) *ALIGNPAD;
			#endif
			PRINTME("--- allocated node data <%i B>\n", nodedatasize );
			PRINTME("--- allocated total data <%.2f MB>", (nodedatasize*_iDataStorage::GetTotalNodes())/1048576.0f );
		}
	}
	void DumpData( const int frequency=10 )
	{
		PRINTME("\nPointList data dumping ::");
		iUint tnodes = _iDataStorage::GetTotalNodes();
		for( iUint x=0; x<tnodes; x++)
		{
			int slot=3;
			if(x%(tnodes/frequency)==0)
			{	
				PRINTME("\npoint<%u>::",x);

				iVector3 hpos (  GetNodeVectorAt< DATA_SLOT_POS >(x) );
				PRINTME("\n --- pos: <%f %f %f>", hpos[0], hpos[1], hpos[2]);

				for(int i=0; i<iPListDataLayout::GetTotalFields(); i++)
				{
					const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];
					if ( this_attr.type == 2 ) {
						PRINTME("\n --- slot <%f>", *(operator()(x, this_attr.offset)));
					}
					if ( this_attr.type >= 3 ) {
						iVector3 hval ( operator()(x, this_attr.offset) );
						PRINTME("\n --- slot <%f %f %f>",hval[0], hval[1], hval[2]);
					}
				}
			}
		}
		PRINTME("\n\n");
	}


	// build plist from other storage
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Get pointlist from mray baked data //////////////////////////////////////////////////////////
	template<class iSurfel>
	void TransferMISurfelData( iSurfel* iSCache, const int verbose )
	{
		// Get cached data ..
		int cSize = iSCache->GetCacheChunkSize();	// .. in chunks 
		iSurfel::_surfelList ** tlocal_scache = iSCache->GetDataCache();
		
		// Total points in plist
		iUint ntotalpoints=0;
		size_t * totalp_in_chunk = (size_t*) malloc (sizeof(size_t) *cSize);
		ntotalpoints = iSCache->GetCacheChunksPoints( totalp_in_chunk, verbose );

		// Allocate plist pointer (to hold nodes) //////////////////////////////////////////////////
		this->AllocateStorage(ntotalpoints);


		////////////////////////////////////////////////////////////////////////////////////////////
		// chunks in cache
		iUint plistID = 0;
		for (int i=0; i < cSize; i++)
		{
			if(ABORTCALLED) return;

			// chunk dpointers 
			vector<DataType*> slist = tlocal_scache[i]->surfel_list;

			// surfels in chunk
			for( iUint v=0; v < totalp_in_chunk[i]; v++ )
			{
				DataType* plistNode = AllocateNewNode();				// alloc slots for surfel data

				_iNodeAllocator::NodeCopy( plistNode, slist[v] );
#if ! __SSE2_ENHANCED__
				_iNodeAllocator::NodeFillSlots( plistNode );			// init result slots
#endif
				AddNode(plistNode);										// add node to plist
				
				free( slist[v] );
				plistID++;												// global idx
			}
			
			slist.clear();												// clear pointer list
			delete tlocal_scache[i];									// delete mi cache
		}
		////////////////////////////////////////////////////////////////////////////////////////////

		free( totalp_in_chunk );										// free chunck ntot list
	};


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Get pointlist from mray baked data //////////////////////////////////////////////////////////
	template<class iSurfel>
	bool TransferMISurfelDataToMiMap( iSurfel* iSCache, const char * oFile, bool isbinary=true, bool isopen=false, const int verbose=0 )
	{
		// Get cached data ..
		int cSize = iSCache->GetCacheChunkSize();	// .. in chunks 
		iSurfel::_surfelList ** tlocal_scache = iSCache->GetDataCache();
		
		// Total points in plist
		iUint ntotalpoints=0;
		size_t * totalp_in_chunk = (size_t*) malloc (sizeof(size_t) *cSize);
		ntotalpoints = iSCache->GetCacheChunksPoints( totalp_in_chunk, verbose );
		
		////////////////////////////////////////////////////////////////////////////////////////////
		int ndecl = iPListDataLayout::GetTotalFields();
		Map_declaration	odeclaration( 3 );

		// global fields decl
		const _iDataFlag &global_attr = iPListDataLayout::GetGlobalFields()[0];///TODO:ADD:HASGLOBALFIELDS:TO:LAYOUT:!!!!!!!!!!
		odeclaration->add_global_integer(  global_attr.name.c_str() );	//map type, ie. ao, sss, gi
		odeclaration->add_global_integer(  "mapsize" );					//map size

		// local fields decl
		for(int i=0; i<ndecl; i++)
		{
			const _iDataFlag &local_attr = iPListDataLayout::GetLocalFields()[i];

			if(local_attr.type==1)		odeclaration->add_integer( local_attr.name.c_str() );
			else if(local_attr.type==2)	odeclaration->add_scalar( local_attr.name.c_str() );
			else if(local_attr.type==3)	odeclaration->add_vector( local_attr.name.c_str() );
			else						odeclaration->add_color( local_attr.name.c_str() );
		}

		// create the point cloud mi map
		Edit_map		wMap		( odeclaration );
		Map_element		wElement	( odeclaration );

		wMap->set( odeclaration->get_field_id( global_attr.name.c_str() ), global_attr.offset);		//offset used to hold map id
		wMap->set( odeclaration->get_field_id( "mapsize" ), (int)ntotalpoints );					//map total size


		////////////////////////////////////////////////////////////////////////////////////////////
		// chunks in cache
		iUint plistID = 0;
		for (int i=0; i < cSize; i++)
		{
			if(ABORTCALLED) break;

			// chunk dpointers 
			vector<DataType*> slist = tlocal_scache[i]->surfel_list;


			// surfels in chunk
			for( iUint v=0; v < totalp_in_chunk[i]; v++ )
			{
				DataType plistNode[NodeSize];
				_iNodeAllocator::NodeCopy( plistNode, slist[v] );		//original plist layout

				// set position
				iVector3 vPos( plistNode );
				wElement->set_position(	vPos.AsMiVector() );

				for(int i=0; i<ndecl; i++)
				{
					const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];
				
					if(this_attr.type==2){
						DataType hval ( plistNode[ this_attr.offset ] );//use plist layout
						wElement->set( odeclaration->get_field_id ( this_attr.name.c_str() ), hval );
					}
					else if(this_attr.type==3){
						iVector3 hval ( &plistNode[ this_attr.offset ] );
						wElement->set( odeclaration->get_field_id ( this_attr.name.c_str() ), hval.AsMiVector() );
					}
					else if(this_attr.type==4){
						iVector3 hval ( &plistNode[ this_attr.offset ] );
						wElement->set( odeclaration->get_field_id ( this_attr.name.c_str() ), hval.AsMiColor() );
					}
				}

				wMap->append( wElement );	// append to map /////////////////////////////////////////////////////

				free( slist[v] );
				plistID++;												// global idx
			}
			
			slist.clear();												// clear pointer list
			delete tlocal_scache[i];									// delete mi cache
		}
		////////////////////////////////////////////////////////////////////////////////////////////

		free( totalp_in_chunk );										// free chunck ntot list



		// Write map
		Map_status status_map = wMap->consolidate();		// Consolidate map
		if(status_map.is_ok())	
		{
 			status_map = wMap->write ( oFile, isbinary, isopen ? mi::shader_v3::Map_base::/*Open_ASCII*/Open_binary : mi::shader_v3::Map_base::Closed_binary);

			if(!status_map.is_ok()){
				PRINTME("--- Problems saving AO map to disk !!");
				return false;
			}
			
			if(verbose)
			PRINTME( "-----------------------------------------------" );

			return true;

		}else{
			WARNME( "Problems consolidating map" );
			return false;
		}

	};


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Get pointlist from mray baked data //////////////////////////////////////////////////////////
	template<class iSurfel>
	bool TransferMISurfelDataToPTC( iSurfel* iSCache, const char * oFile, const int verbose=0 )
	{
		// Get cached data ..
		int cSize = iSCache->GetCacheChunkSize();	// .. in chunks 
		iSurfel::_surfelList ** tlocal_scache = iSCache->GetDataCache();
		
		// Total points in plist
		iUint ntotalpoints=0;
		size_t * totalp_in_chunk = (size_t*) malloc (sizeof(size_t) *cSize);
		ntotalpoints = iSCache->GetCacheChunksPoints( totalp_in_chunk, verbose );
		

		////////////////////////////////////////////////////////////////////////////////////////////
		Partio::ParticlesDataMutable* ioList	= Partio::create();	//create partio file plus default attr
		Partio::ParticleAttribute posAttr		= ioList->addAttribute	("position",	Partio::VECTOR,3);

		// get particles attributes
		int ndecl = iPListDataLayout::GetTotalFields();
		for(int i=0; i<ndecl; i++)
		{
			const _iDataFlag &local_attr = iPListDataLayout::GetLocalFields()[i];

			if(local_attr.type==1)			ioList->addAttribute(local_attr.name.c_str(), Partio::INT,1);
			else if(local_attr.type==2)		ioList->addAttribute(local_attr.name.c_str(), Partio::FLOAT,1);
			else if(local_attr.type==3)		ioList->addAttribute(local_attr.name.c_str(), Partio::VECTOR,3);
			else /*color*/					ioList->addAttribute(local_attr.name.c_str(), Partio::FLOAT,3);
		}


		////////////////////////////////////////////////////////////////////////////////////////////
		// chunks in cache
		bool abortCalled = false;
		iUint particleIdx = 0;
		for (int i=0; i < cSize; i++)
		{
			if(ABORTCALLED){
				abortCalled = true;
				break;
			}

			// chunk dpointers 
			vector<DataType*> slist = tlocal_scache[i]->surfel_list;


			// surfels in chunk
			for( iUint v=0; v < totalp_in_chunk[i]; v++ )
			{

				DataType plistNode[NodeSize];							//hold particle data block
				_iNodeAllocator::NodeCopy( plistNode, slist[v] );		//original plist layout

				Partio::ParticleIndex index =ioList->addParticle();		//add particle

				// set position
				float* pos	=	ioList->dataWrite<float>( posAttr,	particleIdx );
				pos[0] = plistNode[0];
				pos[1] = plistNode[1];
				pos[2] = plistNode[2];


				for(int i=0; i<ndecl; i++)
				{
					const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];
				
					Partio::ParticleAttribute partioAttr;				//hold attr info
					ioList->attributeInfo( this_attr.name.c_str(), partioAttr);

					if(this_attr.type==2){
						float* hfloat	=	ioList->dataWrite<float>( partioAttr,	particleIdx );
						hfloat[0]		=	plistNode[ this_attr.offset ];
					}
					else if(this_attr.type==3){
						float* hvector	=	ioList->dataWrite<float>( partioAttr,	particleIdx );
						iVector3 hval ( &plistNode[ this_attr.offset ] );
						hvector[0] = hval[ 0 ];
						hvector[1] = hval[ 1 ];
						hvector[2] = hval[ 2 ];
					}
					else if(this_attr.type==4){
						float* hcolor	=	ioList->dataWrite<float>( partioAttr,	particleIdx );
						iVector3 hval ( &plistNode[ this_attr.offset ] );
						hcolor[0] = hval[ 0 ];
						hcolor[1] = hval[ 1 ];
						hcolor[2] = hval[ 2 ];
					}
				}

				free( slist[v] );
				particleIdx++;											// global idx
			}
			
			slist.clear();												// clear pointer list
			delete tlocal_scache[i];									// delete mi cache
		}
		////////////////////////////////////////////////////////////////////////////////////////////

		free( totalp_in_chunk );										// free chunck ntot list


		// Write map
		if(!abortCalled)
		{
			Partio::write( oFile, *ioList);
			ioList->release();
				return	true;
		}else	return	false;
	};


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Read data from open binary miMap file ///////////////////////////////////////////////////////
	bool ReadMIMapData( const char * iFile, const int iVerbose=0 )
	{

		int verbose = iVerbose;
		if(verbose)PRINTME("\n// PointList module //////////////////////////////////////////////////"
		"\n--- reading file .. '%s'", iFile);


		// Open miMap 
		FILE    *ifp = fopen ( iFile , "rb" );
		if(!ifp){ 
			PRINTME ("\nFailed reading file ... aborting\n" );
			return 0;
		}


		////////////////////////////////////////////////////////////////////////////////////////////
		// MI Open Binary Map //////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////
		// the header string is made of:
		//
		// - the magic string for the binary map files	... "mi=MAP"
		// - the marker of the open binary format		... "ob1"
		// - the dimension of the map					... "dim3"
		////////////////////////////////////////////////////////////////////////////////////////////
		// - all the fields declaration strings			...
		//													-- "i" - integer
		//													-- "s" - scalar
		//													-- "c" - color
		//													-- "v" - vector
		//													-- "t" - transform
		//													-- "$" - string
		//
		//													-- the "g" character,
		//														is an optional flag 
		//														that specifies that the field 
		//														is a global one;
		//													-- the field name,
		//														with quote characters
		//														around the name!
		// - a final terminating null character ('\0')	////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////

		// magic string for the binary map files
		char    mi_magic_string[7];
		fread ( mi_magic_string , 1 , 6 , ifp );
		mi_magic_string[6] = '\0';	// .. a C-string is an array of chars terminated by a zero value char

		if ( strcmp ( mi_magic_string , "mi=MAP" ) != 0 )	{
			PRINTME ( "'%s' is not an Open Binary MAP file ... aborting.\n" , iFile );
			return 0;
		}
		else{
			if(verbose)PRINTME("\n\nDetected open binary map file ...\n");
			if(verbose>=3)PRINTME("--- magic string: <%s> \n",  mi_magic_string);
		}

		// marker of the open binary format
		char    marker_magic_string[4];
		fread ( marker_magic_string , 1 , 3 , ifp );
		marker_magic_string[3] = '\0';

		if ( strcmp ( marker_magic_string , "ob1" ) != 0 )	{
			PRINTME ( "'%s' is not an Open Binary MAP file ... aborting.\n" , iFile );
			return 0;
		}
		else
			if(verbose>=3)PRINTME("--- marker string: <%s> \n",  marker_magic_string);

		// map dimensions
		char    dim_magic_string[5];
		fread ( dim_magic_string , 1 , 4 , ifp );
		dim_magic_string[4] = '\0';

		if ( strcmp ( dim_magic_string , "dim3" ) != 0 )	{
			PRINTME ( "'%s' is not an 3D Open Binary MAP file ... aborting.\n" , iFile );
			return 0;
		}
		else
			if(verbose>=3)PRINTME("--- dimensions string: <%s> \n",  dim_magic_string);


		// **Global field ..
		if(verbose)PRINTME("\nGlobal Fields ...\n");

		//char fType[2];
		//fread(fType, 1, 1, ifp);
		//fType[1] = '\0';



		// **Local fields entries
		if(verbose)PRINTME("\nFields ...\n");
		//////////////////////////////////////////////////////////////////////////////////////////////////
		int nodeoffset			= 0;
		int lastfield			= 0;
		bool isGlobalField		= false;
		bool hasGlobalFields	= false;

		char finalChar			= 'x';
		//mi=MAPob1dim3 ig"maptype" ig"mapsize" v"dir" s"area" v"t0" v"t1" v"t2" s"ao"

		do
		{
			// chech field type //////////////////////////////////////////////////////////////////////////
			char fType[2];
			fread(fType, 1, 1, ifp);
			fType[1] = '\0';

			if(strcmp ( fType , "\0" ) == 0){
				finalChar = '\0';
				if(verbose>=4)PRINTME("local fields finished ..\n");
				break;
			}


			if (	strcmp ( fType , "v" ) != 0 &&
					strcmp ( fType , "c" ) != 0 &&
					strcmp ( fType , "s" ) != 0 &&
					strcmp ( fType , "t" ) != 0 &&
					strcmp ( fType , "i" ) != 0 &&
					strcmp ( fType , "$" ) != 0  )	
			{
				PRINTME ( "field type not recognised .. aborting\n");
				std::cout << fType << endl;
				break;
			}


			char gType[2];
			fread(gType, 1, 1, ifp);		//continue reading ...
			gType[1] = '\0';

			if(strcmp( gType, "g" ) == 0)	//to see if it's a global field
			{
				isGlobalField	= true;		
				hasGlobalFields	= true;
			}else isGlobalField	= false;

			// get field name ////////////////////////////////////////////////////////////////////////////
			char *fName = (char*) malloc (sizeof(char));
			if(verbose>=4)PRINTME("--- allocated <%d> bytes\n", int( strlen(fName)) );

			if( isGlobalField )
				fread( fName, 1, 1, ifp);
			else
				fName[0] = gType[0];

			char *more_chars;

			char holdChar[2];
			holdChar[0] = fName[0];
			holdChar[1] = '\0';
			if(verbose>=4)PRINTME (" -> for char < %s > \n" , holdChar );

			if ( strcmp ( holdChar , "\"" ) != 0 )	{
				PRINTME ( "invalid declaration .. aborting\n");
				std::cout << holdChar << endl;
				return 0;
			}

			int count = 1;
			do
			{
				count++;

				more_chars = (char*) realloc (fName, count * sizeof(char));	//TODO:fread !?
				if(verbose>=4)PRINTME("--- allocated <%d> bytes\n", int( strlen(more_chars)) );

				if (more_chars!=NULL) 
				{
					fread( holdChar, 1, 1, ifp);

					fName = more_chars;
					fName[count-1] = holdChar[0];

					holdChar[1] = '\0';
					if(verbose>=4)PRINTME ("--- for char < %s > \n" , holdChar );
				}
			} while (holdChar[0] != '\"');


			// consolidate name string
			count++;
			more_chars = (char*) realloc (fName, ( count) * sizeof(char));
			fName = more_chars;
			fName[count-1] = '\0';

			if(verbose>=3)PRINTME ("--- field type <%s>, with name < %s > \n" , fType, fName );


			// Store data layout /////////////////////////////////////////////////////////////////////////
			if(iPListDataLayout::GetTotalFields()!=0 && GetTotalNodes()!=0)  
											// if we have already a layout for this plist it may have been already partially
											// filled in .. so no need to re-add a layout, just check it's consistent
			{
				///TODO:CHECK:FOR:LAYOUT:CONSISTENCY
				//
			}else
			{
				if( isGlobalField )
				{
					_iDataFlag globalflag;

					//remove quotes around name
					string s ( fName );
					s.erase( remove( s.begin(), s.end(), '\"' ), s.end() );

					globalflag.name = s;
					globalflag.type = iPListDataLayout::GetFieldType(fType);

					iPListDataLayout::AddGlobalField( globalflag );

				} else
				{
					_iDataFlag localFlag;

					//remove quotes around name
					string s ( fName );
					s.erase( remove( s.begin(), s.end(), '\"' ), s.end() );
					localFlag.name = s;

					//localFlag.name = fName;
					localFlag.type = iPListDataLayout::GetFieldType(fType);

					_iDataFlag lastflag;
					if(lastfield){
						lastflag			= iPListDataLayout::GetLocalFields()[lastfield-1];
						localFlag.offset	= lastflag.offset + iPListDataLayout::GetFieldDataOffset( lastflag.type );
					}else
					{
						localFlag.offset	= iPListDataLayout::GetFieldDataOffset( localFlag.type );
					}

					iPListDataLayout::AddLocalField( localFlag );
					lastfield++;
				}
			}

			////////////////////////////////////////////////////////////////////////////////////////////////////
			if(verbose>=4)PRINTME("--- freeing <%d> bytes\n", int( strlen(fName)) );
			free (fName);

		} while(finalChar!= '\0');



		/// DEBUG //////////////////////////////////////////////////////////////////////////////////////////////
		std::cout << "\n" << endl;
		iPListDataLayout::PrintDataLayout();
		std::cout << "\n" << endl;
		/// DEBUG //////////////////////////////////////////////////////////////////////////////////////////////


		// **Read global fields ////////////////////////////////////////////////////////////////////////////////
		iUint mapsize = 0;

		if( hasGlobalFields )
		for(int i=0; i<iPListDataLayout::GetTotalFields(true); i++)
		{

			_iDataFlag this_attr = iPListDataLayout::GetGlobalFields()[i];
			if ( this_attr.type == 1 )	{
				// integer
				int	v;
				//fwrite ( &v , sizeof ( int ) , 1 , ifp );
				fread( &v, 1, sizeof(int), ifp);
				this_attr.offset = v;
			}

			if( this_attr.name=="mapsize" ) mapsize = this_attr.offset;	// we stored mapsize in offset member 
		}


		// **Read values ///////////////////////////////////////////////////////////////////////////////////////
		if(verbose)PRINTME("\nReading binary local data ...\n");	////////////////////////////////////////////

		int verboseStep = 10000;
		size_t result;

		AllocateStorage( mapsize );	/// ////////////////////////////////////////////////////////////////////////

		int e = 1;
		do{	/// /////////////////////////////////////////////////////////////////////////////////////////////////
		//for(iUint it=0; it<3244356U; it++){

		if(e%verboseStep==0)
		if(verbose>=3)PRINTME("\n--- <%d> element ..\n", e);

		// Hold binary mimap element(node) data
		DataType* plistNode = AllocateNewNode();	// alloc slots for surfel data /////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////

		iVector3 tVec;	//hold vals from bfile

		// Check if begining of element is garbage .. ie. EOF
		result = fread( &tVec[0], 1, sizeof(float), ifp );
		if(result!=sizeof(float)) break;	// we break here


		// Position first ..
		fread( &tVec[1], 1, sizeof(float), ifp);
		fread( &tVec[2], 1, sizeof(float), ifp);

		////////////////////////////
		StoreFieldData( &plistNode[DATA_SLOT_POS], tVec );
		if(e%verboseStep==0 && verbose>=3) std::cout << " -- position: " << tVec;		//TODO: REMOVE std::cout!


#ifdef BETTEROPTIMIZE

		//unroll the un-optimized below for-loop into the following ?! .. 
		iVector3 dirVec;
		DataType areaF;
		iVector3 GI;

		//dir
		fread( &dirVec, 1, sizeof(float)*3, ifp);
		StoreFieldData( &plistNode[DATA_SLOT_DIR], dirVec );

		//area
		fread( &areaF, 1, sizeof(DataType), ifp);
		StoreFieldData( &plistNode[DATA_SLOT_AREA], areaF );

		//ao
#if	__SSE2_ENHANCED__
		fread( &GI, 1, sizeof(DataType)*4, ifp);
#else
		fread( &GI, 1, sizeof(DataType)*3, ifp);
		fread( &areaF, 1, sizeof(DataType), ifp);//dummy 4th for color
#endif
		StoreFieldData( &plistNode[NODERESULTSLOT], GI );
		//seems no penalty if storing it interleaved so ...
		//StoreFieldData( &plistNode[DATA_SLOT_POS], tVec );
		//StoreFieldData( &plistNode[DATA_SLOT_DIR], dirVec );
		//StoreFieldData( &plistNode[DATA_SLOT_AREA], areaF );
		//StoreFieldData( &plistNode[NODERESULTSLOT], aoF );

#else

		// .. then all the others //////////////////////////////////////////////////////////////////////////////
		for(int i=0; i<iPListDataLayout::GetTotalFields(); i++)
		{
			const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];

	    	if ( this_attr.type == 1 )	{
				// integer
				//int	v = *(operator()(n, this_attr.offset));
				//fwrite ( &v , sizeof ( int ) , 1 , ofp );
	    	}
	    	else if ( this_attr.type == 2 ) {
				// scalar
				DataType fval;
				fread( &fval, 1, sizeof(DataType), ifp);

				////////////////////
				#if	__SSE2_ENHANCED__
				_mm_store_ps( &plistNode[this_attr.offset], _mm_setr_ps(fval, 1.f, 1.f, 1.f) );
				#else	
				plistNode[this_attr.offset] = fval;
				#endif

				if(e%verboseStep==0 && verbose>=3)
				PRINTME("\n -- %s: <%f> %i", this_attr.name.c_str(),fval, this_attr.offset );
	    	}
	    	else if ( this_attr.type == 3 ) {
				// vector, three floats - one after the other
				fread( &tVec[0], 1, sizeof(float), ifp);
				fread( &tVec[1], 1, sizeof(float), ifp);
				fread( &tVec[2], 1, sizeof(float), ifp);

				#if	__SSE2_ENHANCED__
				_mm_store_ps( &plistNode[this_attr.offset], tVec );
				#else
				plistNode[this_attr.offset]		= tVec[0];
				plistNode[this_attr.offset+1]	= tVec[1];
				plistNode[this_attr.offset+2]	= tVec[2];
				#endif
				if(e%verboseStep==0 && verbose>=3) 
				PRINTME("\n -- %s: <%f,%f,%f> %i", this_attr.name.c_str(),tVec[0],tVec[1],tVec[2], this_attr.offset);

	    	}else if ( this_attr.type == 4 ) {
				// color, four floats - one after the other
				iColor3 tCol;	//hold vals from bfile

				fread( &tCol[0], 1, sizeof(float), ifp);
				fread( &tCol[1], 1, sizeof(float), ifp);
				fread( &tCol[2], 1, sizeof(float), ifp);
				#if	__SSE2_ENHANCED__
				fread( &tCol[3], 1, sizeof(float), ifp);
				#else
				DataType fval;	//dummy 4th for color
				fread( &fval, 1, sizeof(float), ifp);
				#endif

				#if	__SSE2_ENHANCED__
				_mm_store_ps( &plistNode[this_attr.offset], tCol );
				#else
				plistNode[this_attr.offset]		= tCol[0];
				plistNode[this_attr.offset+1]	= tCol[1];
				plistNode[this_attr.offset+2]	= tCol[2];
				#endif

				if(e%verboseStep==0 && verbose>=3) 
				PRINTME("\n -- %s: <%f,%f,%f> %i", this_attr.name.c_str(),tCol[0],tCol[1],tCol[2], this_attr.offset);
	    	}				
		}
#endif

		////////////////////////////////////////////////////////////////////////////////////////////
		AddNode(plistNode);						// add Node to PointList ///////////////////////////

		e++;
		}while(verboseStep != 1);				// end of local fields read ////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		if( IsStorageConsistent() )
		{
			//_iDataStorage::SetTotalNodes(e-1);	// set total points for pointlist //////////////////
			if(verbose>=3) PRINTME("\n\n--- points : %u\n", _iDataStorage::GetTotalNodes());
			return 1;
		}
		else
		{
			PRINTME("\n\nSomething bad happened (mem alloc related, most probably) .. aborting.");
			_iDataStorage::SetTotalNodes( 0 );
			return 0;
		}

	};

	
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Write data for open binary miMap file ///////////////////////////////////////////////////////
	bool WriteMIMapData( const char	*oFile, const int iVerbose=0 )
	{

	    // creates/overwrites a binary file
	    FILE    *ofp = fopen ( oFile , "wb" );

	    // the magic string for binary map files
	    fprintf ( ofp , "mi=MAP" );

	    // the open binary marker ("ob"), followed by the file version (1)
	    fprintf ( ofp , "ob1" );

	    // it's a 3-dimensional map
	    fprintf ( ofp , "dim3" );


	    // Fields declaration strings ..
	    // one after the other with no separation between them.
	    // The format is: <field type>[<dim>][g]"<field name"


	    // declare global fields
	    for ( iUint i = 0 ; i < iPListDataLayout::GetGlobalFields().size() ; ++i )
	    {
			const _iDataFlag & this_field = iPListDataLayout::GetGlobalFields()[i];
			iPListDataLayout::WriteField(ofp, this_field, true);
	    }

	    // declare local fields
	    for ( iUint i = 0 ; i < iPListDataLayout::GetLocalFields().size(); ++i )
	    {
			const _iDataFlag & this_field = iPListDataLayout::GetLocalFields()[i];
			iPListDataLayout::WriteField(ofp, this_field);
	    }

	    // writes a null character that terminates the declaration string
	    fputc ( '\0' , ofp );


	    // Fields definitions ..

	    // define global fields
	    if ( ! iPListDataLayout::GetGlobalFields().empty())	 {
		for ( iUint i = 0 ; i < iPListDataLayout::GetGlobalFields().size() ; ++i )
		{
		    //const _iDataFlag & this_attr = iPListDataLayout::GetGlobalFields()[i];

		    /*
		    if ( this_attr.type == 1 )
		    {
				// integer
				int	v = this_attr.int_values[0];
				fwrite ( &v , sizeof ( int ) , 1 , ofp );
		    }
		    else if ( this_attr.type == 2 )
		    {
				// scalar
				float	v = float ( this_attr.double_values[0] );
				fwrite ( &v , sizeof ( float ) , 1 , ofp );
		    }else
		    {
				// vector( 3 floats - one after the other )
				Vector	v = this_attr.vector_values[0];
				fwrite ( &v.x , sizeof ( float ) , 1 , ofp );
				fwrite ( &v.y , sizeof ( float ) , 1 , ofp );
				fwrite ( &v.z , sizeof ( float ) , 1 , ofp );
		    }
		    */
		}}


	    // define local fields
	    for ( iUint n = 0 ; n < _iDataStorage::GetTotalNodes(); ++n )
	    {
			// first the position...
	    	iVector3 this_position = this->GetNodeVectorAt<DATA_SLOT_POS>(n);
	    	fwrite ( &this_position[0], sizeof ( DataType ) , 1 , ofp );
	    	fwrite ( &this_position[1], sizeof ( DataType ) , 1 , ofp );
	    	fwrite ( &this_position[2], sizeof ( DataType ) , 1 , ofp );

	    	// ... then all other fields values
	    	for ( iUint i = 0 ; i < iPListDataLayout::GetLocalFields().size(); ++i )
	    	{
	    	    const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];

	    	    if ( this_attr.type == 1 )	{
					// integer
					int	v = *(operator()(n, this_attr.offset));
					fwrite ( &v , sizeof ( int ) , 1 , ofp );
	    	    }
	    	    else if ( this_attr.type == 2 ) {
					// scalar
	    	 		DataType v = *(operator()(n, this_attr.offset));
					fwrite ( &v , sizeof ( DataType ) , 1 , ofp );
	    	    }
	    	    else{
					// vector, three floats - one after the other
					iVector3 v ( operator()(n, this_attr.offset) );
					fwrite ( &v[0], sizeof ( DataType ) , 1 , ofp );
					fwrite ( &v[1], sizeof ( DataType ) , 1 , ofp );
					fwrite ( &v[2], sizeof ( DataType ) , 1 , ofp );
	    	    }
	    	}
	    }

		if(iVerbose)
	    PRINTME("\nWriting pointcloud to open binary MIMap format <%s>\n", oFile);
		return fclose ( ofp );
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Read data for closed/open ascii/binary miMap with mray api /////////////////////////////////
	bool ReadMIMapDataMRay( const char * iFile, const int iVerbose=0 )
	{
		// reads the map from the file
		Access_map iMap( iFile );

		// gets a copy of the map declaration
		Map_declaration	iDecl( iMap );

		// validate map
		iUint mapsize = iMap->size();
		iUint mapstepverbose = mapsize/5;
		if(iMap->is_empty()){ ERRORME( "--- map -> is empty .. aborting !" );return false; }
		else if(iVerbose>=2){ PRINTME( "--- n. of elements: %u" , iMap->size() ); }

		int ndecl = iPListDataLayout::GetTotalFields();
		for(int i=0; i<ndecl; i++)					//check it has the needed element fields
		{
			const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];

			if(iVerbose>=2)PRINTME("--- retrieved decl field .. %s", this_attr.name.c_str()); 
			if( ! iDecl->has_field(this_attr.name.c_str()) ) return false;	//if not, return
		}

		// allocate pointlist storage
		this->AllocateStorage( iMap->size() );

		// position is mandatory
		miVector vPos;

		// transfer data from miMap to pointlist
		Access_map_iterator	it ( iMap );
		for ( ; ! it->at_end () ; it->next () )
		{
			if (mi_par_aborted()) break;

			DataType* plistNode = AllocateNewNode();	// alloc plist node


			it->get_position( vPos );
			SetNodeVectorAt< DATA_SLOT_POS >(plistNode, iVector3(vPos));

			for(int i=0; i<ndecl; i++)
			{
				const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];


				if(this_attr.type==2){					//scalar

					DataType hval;
					it->get( iDecl->get_field_id( this_attr.name.c_str() ), hval );
					StoreFieldData( &plistNode[this_attr.offset], hval );
				}
				else if(this_attr.type==3){				//vector
					miVector hval;
					it->get( iDecl->get_field_id( this_attr.name.c_str() ), hval );
					StoreFieldData( &plistNode[this_attr.offset], hval );
				}
				else if(this_attr.type==4){				//color
					miColor hval;
					it->get( iDecl->get_field_id( this_attr.name.c_str() ), hval );
					StoreFieldData( &plistNode[this_attr.offset], hval );
				}
			}

			AddNode(plistNode);	/// ////////////////////////////////////////////////////
		}
		
		return true;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	// Write data for closed/open ascii/binary miMap with mray api /////////////////////////////////
	bool WriteMIMapDataMRay( const char * oFile, bool isbinary=true, bool isopen=false, const int iVerbose=0 )
	{
		///////////////////////////////////////////////
		int ndecl = iPListDataLayout::GetTotalFields();
		Map_declaration	odeclaration( 3 );
		

		const _iDataFlag &global_attr = iPListDataLayout::GetGlobalFields()[0];
		odeclaration->add_global_integer(  global_attr.name.c_str() );	//map type, ie. ao, sss, gi
		odeclaration->add_global_integer(  "mapsize" );					//map size

		for(int i=0; i<ndecl; i++)
		{
			const _iDataFlag &local_attr = iPListDataLayout::GetLocalFields()[i];

			if(local_attr.type==1)		odeclaration->add_integer( local_attr.name.c_str() );
			else if(local_attr.type==2)	odeclaration->add_scalar( local_attr.name.c_str() );
			else if(local_attr.type==3)	odeclaration->add_vector( local_attr.name.c_str() );
			else						odeclaration->add_color( local_attr.name.c_str() );
		}


		// create the point cloud mi map
		Edit_map		wMap		( odeclaration );
		Map_element		wElement	( odeclaration );

		wMap->set( odeclaration->get_field_id( global_attr.name.c_str() ), global_attr.offset);		//offset used to hold map id
		wMap->set( odeclaration->get_field_id( "mapsize" ), (int)_iDataStorage::GetTotalNodes() );	//map total size

		// parse pointlist array while setting map elements
		miVector vPos;


#ifdef IMPLEMENTTHIS
		miVector vDir;
		DataType areaF;
		//miVector t0;
		//miVector t1;
		//miVector t2;
		DataType aoF;
		for(unsigned int itx=0; itx<_iDataStorage::GetTotalNodes(); itx++)
		{
			if (ABORTCALLED) break;


			DataType const * const plistnode = _iDataStorage::GetNodePtr(itx);	//prefetch !?

			iVector3 vPos(	_mm_load_ps( &plistnode[DATA_SLOT_POS] ) );
			iVector3 vDir(	_mm_load_ps( &plistnode[DATA_SLOT_DIR] ) );
			iVector3 areaF( _mm_load_ps( &plistnode[DATA_SLOT_AREA] ) );
			//iVector3 t0(	_mm_load_ps( &plistnode[DATA_SLOT_T0] ) );
			//iVector3 t1(	_mm_load_ps( &plistnode[DATA_SLOT_T1] ) );
			//iVector3 t2(	_mm_load_ps( &plistnode[DATA_SLOT_T2] ) );
			iVector3 aoF(	_mm_load_ps( &plistnode[NODERESULTSLOT] ) );

			wElement->set_position(	vPos.AsMiVector() );
			wElement->set( 0, vDir.AsMiVector() );
			wElement->set( 1, areaF[0] );
			//wElement->set( 2, t0.AsMiVector() );
			//wElement->set( 3, t1.AsMiVector() );
			//wElement->set( 4, t2.AsMiVector() );
			wElement->set( 2, aoF[0] );

			wMap->append( wElement );	// append to map /////////////////////////////////////////////////////
		}

#else

		for(unsigned int itx=0; itx<_iDataStorage::GetTotalNodes(); itx++)
		{
			if (ABORTCALLED) break;

			//TODO:GET:THE:WHOLE:NODE:BLOCK:AND:POINT:TO:THAT:INSTEAD:TO:THE:POINTLIST:WITH:OPERATOR():!!!!!!!
			vPos = this->template GetNodeVectorAt<DATA_SLOT_POS>(itx).AsMiVector();
			wElement->set_position(	vPos );

			for(int i=0; i<ndecl; i++)
			{
				const _iDataFlag &this_attr = iPListDataLayout::GetLocalFields()[i];
				
				if(this_attr.type==2){	
					DataType hval ( *(operator()(itx, this_attr.offset)) );
					wElement->set( odeclaration->get_field_id ( this_attr.name.c_str() ), hval );
				}
				else if(this_attr.type==3){
					iVector3 hval ( operator()(itx, this_attr.offset) );
					wElement->set( odeclaration->get_field_id ( this_attr.name.c_str() ), hval.AsMiVector() );
				}
				else if(this_attr.type==4){
					iColor3 hval ( operator()(itx, this_attr.offset) );
					wElement->set( odeclaration->get_field_id ( this_attr.name.c_str() ), hval.AsMiColor() );
				}
			}

			wMap->append( wElement );	// append to map /////////////////////////////////////////////////////
		}

#endif


		// Write map
		Map_status status_map = wMap->consolidate();		// Consolidate map
		if(status_map.is_ok())	
		{
 			status_map = wMap->write ( oFile, isbinary, isopen ? mi::shader_v3::Map_base::/*Open_ASCII*/Open_binary : mi::shader_v3::Map_base::Closed_binary);

			if(!status_map.is_ok()){
				PRINTME("--- Problems saving AO map to disk !!");
				return false;
			}
			
			if(iVerbose)
			PRINTME( "-----------------------------------------------" );

			return true;

		}else{
			WARNME( "Problems consolidating map" );
			return false;
		}
	}


};

}	//namespace end

#endif
