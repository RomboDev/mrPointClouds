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


#ifndef CPC_POINTCONTAINER_H_INCLUDED
#define CPC_POINTCONTAINER_H_INCLUDED

#include <boost/scoped_array.hpp>


namespace cpc
{

/// ------------------------------------------------------------------------------
/// Naive octree for storing a point hierarchy
template< typename DataType, class iPointlist >
class iOctreeGI
{
public:
	/// Tree node
	/// Leaf nodes have npoints > 0, 
	/// specifying the number of child points contained
	struct Node
	{
		Node()
		:	bbox_min(1e30f)
		,	bbox_max(-1e30f)

		,	aggP(0.f)
		,	aggN(0.f)
		,	aggR(0.f)
		,	aggCol(0.f)

		,	npoints(0)

		,	data()
		{
			children[0] = children[1] = children[2] = children[3] = 0;
			children[4] = children[5] = children[6] = children[7] = 0;
		}

		/// Octree node data
		iVector3	bbox_min;				// node bounding box min
		iVector3	bbox_max;				// node bounding box max

		iVector3	aggP;					// aggregated position
		iVector3	aggN;					// normal
		iVector3	aggCol;					// node radiance
		float		aggR;					// and radius

		int npoints;						// Number of child points for the leaf node case

		Node* children[8];					// Child nodes, to be indexed as children[z][y][x]

		boost::scoped_array<float> data;	// Collection of points in leaf
											// The building mechanism stores leaf points 
											// directly in the octree node
	};

	/// Construct tree from array of points
	iOctreeGI():				m_root(0),	m_dataSize(0){}
	~iOctreeGI()				{};

	
	/// Get total node
	int getTotalNodes()			{ return m_totnodes; };

	/// Get root node of tree
	const Node* root()	const	{ return m_root; }

	/// Get number of floats representing each point.
	int dataSize()		const	{ return m_dataSize; }

	/// Clear tree
	void Clear()				{ deleteTree(m_root); }


	/// Build octree from pointlist //////////////////////////////////////////////////////////////////////
	iOctreeGI( iPointlist* pointlist )
	:	m_root(0)
	,	m_dataSize(iPointlist::NODESIZE)
	{
		this->Clear();	///	/////////////////////////////////////// <== !!?
		size_t npoints = pointlist->GetTotalNodes();


		// We make octree bound cubic rather than fitting the point cloud
		// tightly.  This improves the distribution of points in the octree
		// nodes and reduces artifacts when groups of points are aggregated
		// in the internal nodes.
		//
		// If we *don't* do this and we have a rectangular (non-cubic)
		// bound, we end up with a lot more points in one direction inside
		// a node than another.  This means the aggregated averaged point -
		// intended to represent the collection - is in the middle, but
		// with lots of room on either side:
		//
		// +-----------+   ----->    +----/^\----+
		// | o o o o o |  aggregate  |   | . |   |
		// +-----------+             +----\_/----+
		//
		//   <------->                   <--->
		// even distribution           all in middle :(
		//
		// That is, there will be large gaps between neighbouring disks,
		// which gives large transparent gaps in the microrendered surface.

		iVector3	bbox_min;							// node bounding box min
		iVector3	bbox_max;							// node bounding box max

		for(size_t i = 0; i < npoints; ++i)
		{
			iVector3 p ( pointlist->template GetNodeVectorAt<DATA_SLOT_POS>(i) );			
			p.expand_bbox(bbox_min, bbox_max);
		}
		iVector3 d ( bbox_max - bbox_min );				// diagonal
		iVector3 c ( (bbox_min + bbox_max) / 2.f );		// center

		float maxDim2 = ds_MAX(ds_MAX(d.GetX(), d.GetY()), d.GetZ()) / 2;
		bbox_min = c - maxDim2;
		bbox_max = c + maxDim2;


		// Get a ref(workspace) to the actual pointlist
		iPointListSimple workspace;
		workspace.SetStorageArray( pointlist->GetStorageArray() );

		// Build octree
		m_root = makeTree(0, &workspace, npoints, m_dataSize, bbox_min, bbox_max);
	}


	/// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	/// Members 
	Node* m_root;
	int m_dataSize;
	int m_totnodes;


	/// Recursively delete tree, depth first.
	static void deleteTree(Node* n)
	{
		if(!n) return;

		for(int i = 0; i < 8; ++i)
		deleteTree(n->children[i]);

		delete n;
	}


	/// Build a tree node from the given points
	template<class xPointList>
	Node* makeTree(	int					depth, 
					xPointList*			pointlist, 
					size_t				npoints, 
					int					dataSize, 
					const iVector3&		bbox_min,
					const iVector3&		bbox_max
					)
	{
		assert(npoints != 0);
		m_totnodes++;


		// Setup bounding box
		Node* node = new Node;
		node->bbox_min = bbox_min;
		node->bbox_max = bbox_max;

		iVector3 c ( (bbox_min + bbox_max) / 2.f);
		node->npoints = 0;


		// Limit max depth of tree to prevent infinite recursion
		int maxDepth = 24;
		size_t pointsPerLeaf = 8;

		if(npoints <= pointsPerLeaf || depth >= maxDepth)
		{
			// Small number of child points: make this a leaf node
			node->npoints = npoints;

			// Copy over data into node
			node->data.reset(new float[npoints*dataSize]);

			iVector3 sumP(0.f);		iVector3 sumN(0.f);
			iVector3 sumCol(0.f);	float sumA = 0.f;

			for(size_t j = 0; j < npoints; ++j)
			{
				const float* nodedata = pointlist->GetNodePtr( j );

				// copy extra data
				for(int i = 0; i < dataSize; ++i)
				node->data[ j*dataSize + i ] = nodedata[i];

				// compute averages (area weighted)
				float A = pointlist->template GetNodeScalarAt<DATA_SLOT_AREA>( nodedata );
				A *= A;
				sumA	+=	A;
				sumP	+=	A * iVector3(pointlist->template GetNodeVectorAt<DATA_SLOT_POS>( nodedata ));
				sumN	+=	A * iVector3(pointlist->template GetNodeVectorAt<DATA_SLOT_DIR>( nodedata ));
				sumCol	+=	A * iVector3(pointlist->template GetNodeVectorAt<DATA_SLOT_IRRAD>( nodedata ));
			}

			node->aggP		=	1.0f/sumA * sumP;
			node->aggN		=	sumN.normalize();
			node->aggR		=	sqrtf(sumA);
			node->aggCol	=	1.0f/sumA * sumCol;

			return node;
		}
		
		// allocate extra workspace for storing child points (ugh!)
		std::vector< const float*> workspace(8*npoints);
		const float** w = &workspace[0];
		const float** P[8] = {
			w,             w + npoints,   w + 2*npoints, w + 3*npoints,
			w + 4*npoints, w + 5*npoints, w + 6*npoints, w + 7*npoints
		};

		// Partition points into the eight child nodes
		size_t np[8] = {0};
		for(size_t i = 0; i < npoints; ++i)
		{
			const float* p = pointlist->GetNodePtr( i );

			int cellIndex = 4*(p[2] > c.GetZ()) + 2*(p[1] > c.GetY()) + (p[0] > c.GetX());
			P[cellIndex][np[cellIndex]++] = p;
		}

		// Recursively generate child nodes and finalize node
		float sumA = 0;
		iVector3 sumP(0.f);
		iVector3 sumN(0.f);
		iVector3 sumCol(0.f);

		for(int i = 0; i < 8; ++i)
		{
			if(np[i] == 0)
			continue;

			//Box3f bnd;
			iVector3	bnd_bbox_min;			// node bounding box min
			iVector3	bnd_bbox_max;			// node bounding box max
			bnd_bbox_min.SetX( (i     % 2 == 0) ? bbox_min.GetX() : c.GetX() );
			bnd_bbox_min.SetY( ((i/2) % 2 == 0) ? bbox_min.GetY() : c.GetY() );
			bnd_bbox_min.SetZ( ((i/4) % 2 == 0) ? bbox_min.GetZ() : c.GetZ() );
			bnd_bbox_max.SetX( (i     % 2 == 0) ? c.GetX() : bbox_max.GetX() );
			bnd_bbox_max.SetY( ((i/2) % 2 == 0) ? c.GetY() : bbox_max.GetY() );
			bnd_bbox_max.SetZ( ((i/4) % 2 == 0) ? c.GetZ() : bbox_max.GetZ() );

			// Node plist workspace
			iPointListSimple nodeplist;
			nodeplist.SetStorageArray( P[i] );

			//* Make child node
			Node* child = makeTree(depth+1, &nodeplist, np[i], dataSize, bnd_bbox_min, bnd_bbox_max);
			node->children[i] = child;

			// Weighted average with weight = disk surface area
			float A = child->aggR * child->aggR;
			sumA	+=	A;
			sumP	+=	A * child->aggP;
			sumN	+=	A * child->aggN;
			sumCol	+=	A * child->aggCol;
		}

		node->aggP		=	1.0f/sumA * sumP;
		node->aggN		=	sumN.normalize();
		node->aggR		=	sqrtf(sumA);
		node->aggCol	=	1.0f/sumA * sumCol;

		return node;
	}	

};		// iOctreeGI

}		// cpc namespace
#endif	// CPC_POINTCONTAINER_H_INCLUDED
