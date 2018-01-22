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

#ifndef __PCLOUD_DATASTRUCT_IOCTREE__
#define __PCLOUD_DATASTRUCT_IOCTREE__



namespace cpc
{


/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename DType, class iPointlist>
class iOctreeNodeStorage_AO
{
public:

	#if	__SSE2_ENHANCED__ && defined _MSC_VER
	__declspec(align(ALIGNPAD))
	#endif
	struct ocNode		////////////////////////////////////////////////////////////////////////////
	{
		iVector3	centroid;			// average position of the points in node
		iVector3	ncentroid;			// orientation of the node

		iVector3	bbox_min;			// node min bounding box
		iVector3	bbox_max;			// node max bounding box

		iUint		npoints;			// number of data points in this octree node
		iUint		firstpoint;			// index of first data point

		DType		sumAO;				// for AO passes when involved octree nodes
		DType		sumArea;			// node area

		ocNode *	children[8];		// eight children
		bool		isleaf;				// is this node a leaf ?

		////////////////////////////////////////////////////////////////////////////////////////////
		void operator delete(void * p)	// override delete operator
		{
			ocNode * node = (ocNode *)p;

			for(int i=0; i<8; i++)
			{
				if (node->children[i])
				{
					delete(node->children[i]);

					MEMFREE( node->children[i] );
					node->children[i] = NULL;
				}
			}
		}
	};	//end Node struct
	#if	__SSE2_ENHANCED__ && !defined _MSC_VER
	__attribute__ ((aligned (ALIGNPAD)))
	#endif



	///////////////////////////////////////////////////////////////////////////////////////////////////
	static INLINE void NodeComputing( iPointlist * pointlist, ocNode * node )
	{
		int i = node->firstpoint;
		int lastpoint = node->firstpoint + node->npoints;

		iVector3 p				= pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( node->firstpoint );
		node->ncentroid			= pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>( i );
		DType w = node->sumArea = pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( i );

		DType oo		= sqrtf(inv_pi * w);
		DType maxradius	= ds_MAX( (DType)0.0, oo );
				
		node->centroid	= p;
		node->bbox_min	= p;
		node->bbox_max	= p;

		node->sumAO		= 1.0f;
		i++;

		// Compute the bounding box, area, centroid, and power sum
		for ( ; i < lastpoint; i++ )
		{
			p				=  pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( i );
			node->ncentroid += pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>( i );
			w				=  pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( i );

			p.expand_bbox(node->bbox_min, node->bbox_max);

			node->sumArea	+= w;
			DType oo		= sqrtf(inv_pi * w);
			maxradius		= ds_MAX( maxradius, oo );

			node->centroid	+= p;
		}

		w = 1.0f / node->npoints;
		node->centroid *= w;
		node->ncentroid *= w;

		node->bbox_min -= maxradius;
		node->bbox_max += maxradius;
	}
};


/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename DType, class iPointlist>
class iOctreeNodeStorage_SSS
{
public:

	#if	__SSE2_ENHANCED__ && defined _MSC_VER
	__declspec(align(ALIGNPAD))
	#endif
	struct ocNode		////////////////////////////////////////////////////////////////////////////
	{
		iVector3	centroid;			// average position of the points in node
		iColor3		sumPower;			// SSS - sum of rad_t*area of the node pts

		iVector3	bbox_min;			// node bounding box
		iVector3	bbox_max;			// node bounding box

		iUint		npoints;			// number of data points in this octree node
		iUint		firstpoint;			// index of first data point

		DType		sumArea;			// SSS <<

		ocNode *	children[8];		// eight children
		bool		isleaf;				// is this node a leaf ?

		////////////////////////////////////////////////////////////////////////////////////////////
		void operator delete(void * p)	// override delete operator
		{
			ocNode * node = (ocNode *)p;

			for(int i=0; i<8; i++)
			{
				if (node->children[i])
				{
					delete(node->children[i]);

					MEMFREE( node->children[i] );
					node->children[i] = NULL;
				}
			}
		}
	};	//end Node struct
	#if	__SSE2_ENHANCED__ && !defined _MSC_VER
	__attribute__ ((aligned (ALIGNPAD)))
	#endif


	////////////////////////////////////////////////////////////////////////////////////////////////
	static INLINE void NodeComputing( iPointlist * pointlist, ocNode * node )
	{
		int i = node->firstpoint;
		int lastpoint = node->firstpoint + node->npoints;

		iVector3 p				= pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( node->firstpoint );
		DType w = node->sumArea = pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( i );
		node->sumPower			= pointlist->template GetNodeVectorAt< DATA_SLOT_IRRAD >(i) * w;

		DType oo		= sqrtf(inv_pi * w);
		DType maxradius = ds_MAX( (DType)0.0, oo );

		node->centroid	= p;
		node->bbox_min	= p;
		node->bbox_max	= p;
		i++;

		// Compute the bounding box, area, centroid, and power sum
		for ( ; i < lastpoint; i++ )
		{
			p = pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( i );
			w = pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( i );
			node->sumPower += pointlist->template GetNodeVectorAt< DATA_SLOT_IRRAD >(i) * w;	
			
			p.expand_bbox(node->bbox_min, node->bbox_max);

			node->sumArea += w;
			DType oo = sqrtf(inv_pi * w);
			maxradius = ds_MAX( maxradius, oo );

			node->centroid += p;
		}

		w = 1.0f / node->npoints;
		node->centroid *= w;

		node->bbox_min -= maxradius;
		node->bbox_max += maxradius;
	}
};


/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename DType, class iPointlist>
class iOctreeNodeStorage_GI
{
public:

	#if	__SSE2_ENHANCED__ && defined _MSC_VER
	__declspec(align(ALIGNPAD))
	#endif
	struct ocNode		////////////////////////////////////////////////////////////////////////////
	{
		iVector3	bbox_min;			// node bounding box min
		iVector3	bbox_max;			// node bounding box max

		iVector3	aggP;				// aggregated position
		iVector3	aggN;				// normal
		iVector3	aggCol;				// node radiance
		float		aggR;				// and radius

		bool		isleaf;				// is this node a leaf ?
		iUint		npoints;			// n points in node
		iUint		firstpoint;			// index of first point

		ocNode *	children[8];		// eight children

		////////////////////////////////////////////////////////////////////////////////////////////
		void operator delete(void * p)	// override delete operator
		{
			ocNode * node = (ocNode *)p;

			for(int i=0; i<8; i++)
			{
				if (node->children[i])
				{
					delete(node->children[i]);

					MEMFREE( node->children[i] );
					node->children[i] = NULL;
				}
			}
		}
	};	//end Node struct
	#if	__SSE2_ENHANCED__ && !defined _MSC_VER
	__attribute__ ((aligned (ALIGNPAD)))
	#endif


	/// /////////////////////////////////////////////////////////////////////
	static INLINE void NodeComputing( iPointlist * pointlist, ocNode * node )
	{

		float maxradius = 0.0f;
		float maxTrueRadius = 0.0f;
		float sumA = 0.f;
		float sumTrueA = 0.f;
		iVector3 sumP(0.f);
		iVector3 sumN(0.f);
		iVector3 sumCol(0.f);

		int i = node->firstpoint;
		int lastpoint = node->firstpoint + node->npoints;

		const float* nodedata = pointlist->GetNodePtr( i );

		//peel for-loop
		iVector3 iPos	( pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( nodedata ) );
		iVector3 iDir	( pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>( nodedata ) );
		iVector3 AAAA	( pointlist->template GetNodeVectorAt<DATA_SLOT_AREA>( nodedata ) );
		//float A			( pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( nodedata ) );
		iVector3 iIrrad ( pointlist->template GetNodeVectorAt<DATA_SLOT_IRRAD>( nodedata ) );


		float A = AAAA[0];
		float trueA = AAAA[1];

		maxradius = ds_MAX(maxradius, A);
		A *= A;

		//maxTrueRadius = ds_MAX(maxTrueRadius, trueA);
		trueA *= trueA;


		sumP	+=	iPos * A;
		sumN	+=	iDir * A;
		sumA	+=	A;
		//sumCol	+=	iIrrad * A;
		sumTrueA	+=	trueA;
		sumCol	+=	iIrrad * trueA;

		node->bbox_min	= iPos;
		node->bbox_max	= iPos;
		i++;

		// Compute the bounding box, area, centroid, and power sum
		for ( ; i < lastpoint; i++ )
		{
			const float* nodedata = pointlist->GetNodePtr( i );
			
			// compute averages (area weighted)
			iPos	= pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( nodedata );
			iDir	= pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>( nodedata );
			AAAA	= pointlist->template GetNodeVectorAt<DATA_SLOT_AREA>( nodedata );
			//A		= pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( nodedata );
			iIrrad	= pointlist->template GetNodeVectorAt<DATA_SLOT_IRRAD>( nodedata );

			A = AAAA[0]; 
			maxradius = ds_MAX(maxradius, A);
			A *= A;

			trueA = AAAA[1];
			//maxTrueRadius = ds_MAX(maxTrueRadius, trueA);
			trueA *= trueA;

			sumP	+=	iPos * A;
			sumN	+=	iDir * A;
			sumA	+=	A;
			//sumCol	+=	iIrrad * A;
			sumTrueA +=	trueA;
			sumCol	+=	iIrrad * trueA;

			iPos.expand_bbox(node->bbox_min, node->bbox_max);
		}

		node->bbox_min -= maxradius;
		node->bbox_max += maxradius;

#if	__SSE2_ENHANCED__
		node->bbox_max[3] = float( iVector3(node->bbox_max - node->bbox_min).length() / 2.f );
		node->bbox_min[3] = float( sqrtf(sumTrueA) );
#endif

		node->aggP		=	sumP * (1.0f/sumA);
		node->aggN		=	sumN.normalize();
		node->aggR		=	sqrtf(sumA);
		//node->aggCol	=	sumCol * (1.0f/A);
		node->aggCol	=	sumCol * (1.0f/sumTrueA);
	}
};




/// ................................................................................................
#include"_others\iStack.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Octreeclass /////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
template
<
typename DataType, 
class iPointlist, 
template<typename, class> class iNodeStorage = iOctreeNodeStorage_AO
>
class iOctree : public iNodeStorage<DataType, iPointlist>
{

public:

	typedef typename iNodeStorage<DataType, iPointlist>	_iNodeStorageT;
	typedef typename _iNodeStorageT::ocNode				Node;
	

	// Constructors ////////////////////////////////////////////////////////////////////////////////
	iOctree( void ):_totalnodes(0) {};

	iOctree( iPointlist * pointlist ):_totalnodes(0)
	{
		_octree.npoints = pointlist->GetTotalNodes();
		_octree.firstpoint = 0;

		buildOctree(&_octree, 0, pointlist);
	};

	~iOctree(){};		// Destructor //////////////////////////////////////////////////////////////


	// Build octree
	void SplitOctree( iPointlist * pointlist )
	{
		_octree.npoints = pointlist->GetTotalNodes();
		_octree.firstpoint = 0;

		//buildOctree(&_octree, 0, pointlist);
		buildOctreeSequentially(&_octree, pointlist);
	};

	// Root
	const Node* GetRoot( void ) const { return * _octree; };
	Node* GetRoot( void ) { return &_octree; };

	// Get total nodes count
	unsigned int GetSize( void ){ return _totalnodes; };

	// Operators
	const Node* operator->( void ) const { return &_octree; };

	// Delete nodes
	void Clear( void )
	{ 
		if(_totalnodes){
		_totalnodes=0;
		delete &_octree; 
		}
	};

	// Print usefull stuff
	void PrintDataInfo( const int verbose )
	{ 
		if(verbose>=2) PRINTME("--- total nodes<%u>\n", this->GetSize());
		if(verbose>=3) {
			PRINTME("--- allocated node data <%i B>\n", (int)sizeof(Node) );
			PRINTME("--- allocated total data <%.2f MB>", 
			(sizeof(Node) *this->GetSize()) / 1048576.0f );
		}
	}



private: ///////////////////////////////////////////////////////////////////////////////////////////

	Node	_octree;
	iUint	_totalnodes;


	////////////////////////////////////////////////////////////////////////////////////////////////
	void buildOctree( Node *node, const int& level, iPointlist * pointlist )
	{
		if(ABORTCALLED) return;
		_totalnodes++;	// inc total node count


		Node *child;

		float maxradius = 0.0;
		int i, j;
		int ch, c;

		int nptsinchild[8], pos[8];
		int firstpoint = node->firstpoint;
		int lastpoint = firstpoint + node->npoints;

		// Ensure children in child index table
		for (c = 0; c < 2*2*2; c++)
		node->children[c] = NULL;


		////////////////////////////////////////////////////////////////////////////////////////////
		_iNodeStorageT::NodeComputing(pointlist, node);


		// Return if there are less than MAXPTSINNODE (8) points in node or
		// if all the points are in the same position
		if ( node->npoints <= MAXPTSINNODE || ((node->bbox_max - node->bbox_min) < 1e-6f) ) {
			node->isleaf = true;
			return; // no need to split octree node: nothing more to do here	////////////////////
		}


		////////////////////////////////////////////////////////////////////////////////////////////
		node->isleaf = false;

		for (c = 0; c < 8; c++)
		nptsinchild[c] = 0;

		// Determine how many points in the node's pointlist
		// should move to each of the (up to) 8 children
		for (i = firstpoint; i < lastpoint; i++) 
		{
			// Determine which child node the point should be moved to
			c = pointlist->template 
				GetNodeVectorAt<DATA_SLOT_POS>( i ).find_child_bbox(node->bbox_min, node->bbox_max);
			nptsinchild[c]++;
		}

		// Allocate and initialize up to 8 child nodes
		for(c = 0; c < 8; c++) 
		{
			pos[c] = (c > 0) ? pos[c-1] + nptsinchild[c-1] : firstpoint;

			if(nptsinchild[c]) 
			{
				child = (Node*) MEMALLOC(Node,1);
				if (child == NULL) {
					PRINTME("WARNING: Octree doesn't fit in memory ... aborting.\n");
					exit(1);
				}

				child->npoints		= nptsinchild[c];
				child->firstpoint	= pos[c];
				node->children[c]	= child;
			}
		}

		// Swap the points so that the points of each child are consecutive.
		for(ch = 0; ch < 7; ch++) // last child (7) gets trivially done
		{ 
			child = node->children[ch];
			if (!child) continue;
			i = child->firstpoint;
			j = lastpoint - 1;
			while(i < j) 
			{
				do{	// Find the next point to move
					c = pointlist->template 
						GetNodeVectorAt<DATA_SLOT_POS>( i ).find_child_bbox(node->bbox_min, node->bbox_max);
					i++;
				} while (c == ch && i <= j);
				i--;

				
				do{ // Determine which child node the point should be moved to
					c = pointlist->template 
						GetNodeVectorAt<DATA_SLOT_POS>( j ).find_child_bbox(node->bbox_min, node->bbox_max);
					j--;
				} while (c != ch && j >= i);
				j++;

				if (i < j) {
					// Swap points i and j
					pointlist->SwapNodes(i,j);
				}
			}
		}

		// Recurse ////////////////////////////////////////////
		for (c = 0; c < 2*2*2; c++) 
		if (node->children[c])				// Go to child node
		buildOctree(node->children[c], level+1, pointlist);
	}

	

	/// ////////////////////////////////////////////////////////////////////////////////////////////
	void buildOctreeSequentially( Node *root, iPointlist * pointlist )
	{
		iStackX<Node*> istack(256);
		istack.push_back(root);			//push root node to stack

		do
		{
			if(ABORTCALLED) return;
			_totalnodes++;	// inc total node count

			int ch, c;
			iUint i, j;

			// stack ..............................
			Node *node = istack.pop();

			Node *child;
			float maxradius = 0.0f;

			int nptsinchild[8], pos[8];

			iUint npoints		= node->npoints;
			iUint firstpoint	= node->firstpoint;
			iUint lastpoint		= firstpoint + npoints;


			// Init children in child index table
			for (c = 0; c < 2*2*2; c++)
			node->children[c] = NULL;

			
			////////////////////////////////////////////////////////////////////////////////////////////
			_iNodeStorageT::NodeComputing(pointlist, node);


			// Return if there are less than MAXPTSINNODE (8) points in node or
			// if all the points are in the same position
			if ( npoints <= MAXPTSINNODE || ((node->bbox_max - node->bbox_min) < 1e-6f) ) 
			{
				node->isleaf = true;
				continue; // no need to split octree node: nothing more to do here	////////////////////
			}


			////////////////////////////////////////////////////////////////////////////////////////////
			node->isleaf = false;

			for (c = 0; c < 8; c++)
			nptsinchild[c] = 0;

			// Determine how many points in the node's pointlist
			// should move to each of the (up to) 8 children
			for (i = firstpoint; i < lastpoint; i++) 
			{
				// Determine which child node the point should be moved to
				c = pointlist->template 
					GetNodeVectorAt<DATA_SLOT_POS>( i ).find_child_bbox(node->bbox_min, node->bbox_max);
				nptsinchild[c]++;
			}

			// Allocate and initialize up to 8 child nodes
			for(c = 0; c < 8; c++) 
			{
				pos[c] = (c > 0) ? pos[c-1] + nptsinchild[c-1] : firstpoint;

				if(nptsinchild[c]) 
				{
					child = (Node*) MEMALLOC(Node,1);		//allocate node
					if (child == NULL) {
						PRINTME("WARNING: Octree doesn't fit in memory ... aborting.\n");
						exit(1);
					}

					child->npoints		= nptsinchild[c];	//init new node
					child->firstpoint	= pos[c];
					node->children[c]	= child;
				}
			}
			
			// Swap the points so that the points of each child are consecutive
			for(ch = 0; ch < 7; ch++) // last child (7) gets trivially done
			{ 
				child = node->children[ch];
				if (!child) continue;

				i = child->firstpoint;
				j = lastpoint - 1;

				while(i < j)
				{
					do{	// Find the next point to move
						c = pointlist->template 
							GetNodeVectorAt<DATA_SLOT_POS>( i ).find_child_bbox(node->bbox_min, node->bbox_max);
						i++;
					} while (c == ch && i <= j);
					i--;

				
					do{ // Determine which child node the point should be moved to
						c = pointlist->template 
							GetNodeVectorAt<DATA_SLOT_POS>( j ).find_child_bbox(node->bbox_min, node->bbox_max);
						j--;
					} while (c != ch && j >= i);
					j++;

					if (i < j) {
						// Swap points i and j
						pointlist->SwapNodes(i,j);
					}
				}
			}
			

			// 'Recurse' ///////////////////////////////////////////
			for (c = 0; c < 2*2*2; c++)
			{
				if (node->children[c])
				{	// register child node pointer to stack ///////
					istack.push_back(node->children[c]);
				}
			}


		////////////////////////////////////////
		}while( !istack.empty() );	////////////
		istack.clear();/////////////////////////
	}

};		//end class

}		//end namespace
#endif	//end ifndef