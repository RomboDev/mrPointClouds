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
 
 *****************************************************************************

 * Created:	13.11.00
 * Module:	pcloud_Bake_SSScattering
 * Purpose:	Point Based Baker
 *
 * Exports:
 *	pcloud_SSScattering
 *	pcloud_SSScattering_init
 *	pcloud_SSScattering_exit
 *	pcloud_SSScattering_version
 *
 * TODO:
 * -  chech diffuseblend param 1.0 or 0.0 ?? !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

#include "_cpc_mentalray.h"



//////////////////////////////////////////////////////////////////////////////////////////////////////
// parameter struct for the shader
struct pcloud_SSScattering_paras 
{
	miTag			irrad_map;				// irrad map path
	miTag			sss_map;				// sss map path
	miInteger		read_sss;

	miColor			diffuse_albedo;			// SSS parameters:
	miColor			diffuse_meanfreepath;
	miScalar		unitlength;				// multiplier on dmfp
	miScalar		ior;
	miScalar		maxsolidangle;

	miInteger		points_in_lookup;		// points in lookup
	miScalar		multiply_factor;

	miScalar		diffuseblend;

	miColor			fg_bypass;				// fg rays don't work well with pclouds
	miInteger		printProgress;			// verbose
};

// cache structure for the shader
struct pcloud_SSScattering_cache
{
	miTag oMap;		// We get the map from file, then we store in DB init
					// and access it directly from this tag in render loop
	Map_field_id	omap_area_id;
	Map_field_id	omap_sss_id;
	Map_field_id	omap_irrad_id;

	miColor			diffuse_albedo;			// SSS parameters	
	miColor			diffuse_meanfreepath;
	miScalar		unitlength;
	miScalar		ior;
	miScalar		maxsolidangle;

	miInteger		points_in_lookup;		// points in lookup
	miScalar		multiply_factor;

	miScalar		diffuseblend;

	miInteger		printProgress;
	miBoolean		isshaderball;
	miBoolean		disabled;

	miInteger		read_sss;				// for render loop eval

	_pointlistSSS	*pointlist;
	_octreeSSS		*octree;

	float			alphaPTable[steps +1];
	float			A;
	float			aFactor;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
typedef SubSurfaceScattering_Impl<float, _pointlistSSS, iOctreeNodeSSS, pcloud_SSScattering_cache> _sssImplT;


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Shader Initialization
extern "C" DLLEXPORT	int	    pcloud_SSScattering_version(void) {return(1);}
extern "C" DLLEXPORT	void	pcloud_SSScattering_init (
	miState						*state,
	pcloud_SSScattering_paras	*paras,
	miBoolean					*inst_init_req )
{
    if ( ! paras )	{
		*inst_init_req = miTRUE; return;	// shader instance initialization
    }


	// inittialize cache ..
	pcloud_SSScattering_cache* cache = (pcloud_SSScattering_cache*)
										mi_mem_allocate(sizeof(pcloud_SSScattering_cache));

	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	*user = cache;

	// cache parameters
	cache->diffuseblend				= 1.0f - *mi_eval_scalar(&paras->diffuseblend);
	cache->diffuse_albedo			= *mi_eval_color(&paras->diffuse_albedo);
	cache->diffuse_meanfreepath		= *mi_eval_color(&paras->diffuse_meanfreepath);

	cache->unitlength				= *mi_eval_scalar(&paras->unitlength);
	cache->ior						= *mi_eval_scalar(&paras->ior);
	cache->maxsolidangle			= *mi_eval_scalar(&paras->maxsolidangle);

	cache->points_in_lookup			= *mi_eval_integer(&paras->points_in_lookup);
	if(cache->points_in_lookup==0)	cache->points_in_lookup=1;

	cache->multiply_factor			= *mi_eval_scalar(&paras->multiply_factor);
	if(cache->multiply_factor==0)	cache->multiply_factor=1;


	cache->printProgress			= *mi_eval_boolean(&paras->printProgress);

	// check if in hypershader
	cache->isshaderball = miFALSE;
	const miCamera * camera = state->camera;
	if( camera->focal == 1.0 || camera->x_resolution<320){	//TODO::IMRPOVE CHECK HERE !!!!!!!!!!!
		cache->isshaderball = miTRUE;
		return;
	}

	// check if in lightmap stage 
	// called during 2nd rays eval
	miStage stage;
	mi_query(miQ_STAGE, state, 0, &stage);
	if(stage == miSTAGE_LIGHTMAP){
		cache->disabled = miTRUE;
		return;
	}


	if(cache->printProgress){
	mi_info ( "-----------------------------------------------" );
	mi_info ( "--- Reading SSS point cloud" );
	mi_info ( "-----------------------------------------------" );
	}

	cache->disabled = miFALSE;

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// cache sss map as we have it already on disk, so return early
	cache->read_sss = *mi_eval_integer(&paras->read_sss);
	if(cache->read_sss == 1)
	{

		// sss map tag
		miTag	omap_tag = *mi_eval_tag ( &paras->sss_map );
		char* omap_file_name = ( char * ) mi_db_access ( omap_tag );
		mi_db_unpin ( omap_tag );

		Access_map  sss_map ( omap_file_name );

		if(sss_map->is_empty()){ 
			mi_error ( "--- Map -> is empty !" );

			cache->disabled = miTRUE;
			return;
		}
		else{
			if(cache->printProgress){
			mi_info ( "--- Map -> n. of elements: %u" , sss_map->size() );
			mi_info ( "---" );
			}
		}

		// store the map in scene DB
		cache->oMap = sss_map.store();

		Map_declaration	sss_declaration( sss_map );
		cache->omap_sss_id = sss_declaration->get_field_id ( "sss" );

		return;	// return early as we're shading directly from pcloud ////////////////////////////////
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////
	// Irradiance pcloud map
    if (cache->printProgress) 
		mi_info("--- Pass 0: Reading irradiance map\n");


	// SSS miMap tag filepath
	miTag	imap_tag = *mi_eval_tag ( &paras->sss_map );
	char* imap_file_name = ( char * ) mi_db_access ( imap_tag );
	mi_db_unpin ( imap_tag );


	// Initialize pointlist //////////////////////////////////////////////////////////////////////////
	_pointlistSSS *_pointlist = new _pointlistSSS();	// pointlist
	_sssImplT::LayoutPointlistData(_pointlist);			// data layout


	//////////////////////////////////////////////////////////////////////////////////////////////////
	// Point List data
	if(cache->printProgress)  mi_info("Pass 1: Consolidating pointlist\n");
	if( _pointlist->ReadMIMapDataMRay(imap_file_name, cache->printProgress) )
	{
		_pointlist->PrintDataInfo(cache->printProgress);
		if(cache->printProgress) _pointlist->DumpData();

		
		if(cache->printProgress)  mi_info("Pass 2: Building octree\n");
		_octreeSSS *_octree = new _octreeSSS();			// octree ..
		_octree->SplitOctree( _pointlist );				// splitting
		_octree->PrintDataInfo(cache->printProgress);
	
		cache->pointlist	= _pointlist;				// point it to local cache
		cache->octree		= _octree;
		
	}else
	{
		// Access miMap
		Access_map  sss_map ( imap_file_name );
		cache->oMap = sss_map.store();

		Map_declaration			sss_declaration( sss_map );
		cache->omap_sss_id =	sss_declaration->get_field_id ( "sss" );

		cache->read_sss = 1;
		return;
	}


	if(cache->read_sss == 2)
	{	
		float A = _sssImplT::GetAlphaForTable( cache->ior );
		_sssImplT::FillAlphaPrimeTable(cache->alphaPTable, A);
		cache->A = 1.0f + 1.333333f * A;

		// cache map as it's used for diffuse blending
		if(cache->diffuseblend != 1.0f)
		{
			if (cache->printProgress){
				mi_info("--- Pass 3: Storing Diffuse map\n");
				mi_info("--- Pass 4: Evaluating SSS on iCache\n");
				mi_info("--- ...\n");
			}


			Access_map  map ( imap_file_name );
			Map_declaration	ideclaration( map );

			cache->oMap = map.store();
			cache->omap_irrad_id = ideclaration->get_field_id ( "irrad" );

		}else
		{
			if (cache->printProgress){
			mi_info("--- Pass 3: Evaluating SSS on octree\n");
			mi_info("--- ...\n");
			}
		}
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Shader Exit
extern "C" DLLEXPORT void pcloud_SSScattering_exit(
	miState * state,
	pcloud_SSScattering_paras * paras )
{  
	if (!paras) return;

	void **user; 
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_SSScattering_cache* cache = static_cast< pcloud_SSScattering_cache* >(*user);

	if(!cache->isshaderball)
	{
		if(cache->read_sss < 2)
		{
			if(cache->oMap) mi_db_delete(cache->oMap);	// delete data map
		}else
		{
			if(cache->diffuseblend != 1.0f)
			if(cache->oMap) 
			mi_db_delete(cache->oMap);					// delete data map

			cache->pointlist->FreeStorage();
			cache->octree->Clear();
			delete cache->pointlist;
			delete cache->octree;
			cache->pointlist = miNULLTAG;
			cache->octree = miNULLTAG;
		}
	}
	mi_mem_release( *user );							// delete user cache
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Shader Implementation
extern "C" DLLEXPORT	miBoolean   pcloud_SSScattering (
	miColor	    *result,
	miState	    *state,
	struct pcloud_SSScattering_paras	*paras)
{
    result->r = result->g = result->b = 0.0f;
    result->a = 1.0f;


	// * Get shader cache
	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_SSScattering_cache* cache = static_cast< pcloud_SSScattering_cache* >(*user);

	// exit earlier if ...  
	if(cache->isshaderball || cache->disabled) 
	{
		if(mi_is_connected(state, &paras->fg_bypass))
		*result = *mi_eval_color(&paras->fg_bypass);

		return miTRUE;	// shaderball render
	}

	if(state->type==miRAY_FINALGATHER)
	{
		if(mi_is_connected(state, &paras->fg_bypass))
		*result = *mi_eval_color(&paras->fg_bypass);

		return miTRUE;
	}


	// Read pre-computed SSS map /////////////////////////////////////////////////////////////////////
	if(cache->read_sss < 2)
	{
		// ** Setup map lookup
		if( cache->oMap==miNULLTAG ){
		
			mi_error("No map saved to DB or invalid map tag !!");
			return miFALSE;
		}

		// map accessor
		Access_map  map = cache->oMap;
		if( map->is_empty() ){
			
			mi_error("Map is empty !!");
			return miFALSE;
		}

		// attach the lookup to the map
		Map_lookup	result_lookup ( map );
		{
			// runs a lookup inside the map
			result_lookup->search ( state->point, cache->points_in_lookup );

			// accumulates the colors of all the elements which have been found
			miUint		num = 0;
			miColor		element_sss;
			for ( ; ! result_lookup->at_end () ; result_lookup->next () )	{

				// retrieve ssdiffusion data.
				result_lookup->get ( cache->omap_sss_id , element_sss );

				result->r += element_sss.r;
				result->g += element_sss.g;
				result->b += element_sss.b;
			}

			// keeps track of how many elements were used in all
			num = result_lookup->size ();

			// divides by the total number of elements to get the average
			if ( num > 0 )  {
				result->r /= ( float ) num;
				result->g /= ( float ) num;
				result->b /= ( float ) num;
			}
		}

	}
	
	// Compute SSS using octree as icache   //////////////////////////////////////////////////////////
	else if(cache->read_sss == 2)			// eval everything
	{

		iVector3 ppos( state->point );
		iVector3 diff_albedo( *mi_eval_color(&paras->diffuse_albedo) );
		iVector3 diff_mfp( *mi_eval_color(&paras->diffuse_meanfreepath) );
		
		// compute sss for the sample point //////////////////////////////////////////////////////////
		iVector3 result_c = 
		_sssImplT::DoScattering
		(	
			ppos,

			diff_albedo,
			diff_mfp, 

			cache->unitlength, 
			cache->maxsolidangle, 

			cache->alphaPTable,
			cache->A,

			cache->pointlist,
			cache->octree->GetRoot()  
		);

		result->r = result_c[0];
		result->g = result_c[1];
		result->b = result_c[2];



		// lookup irrad map to blend it with diffuse /////////////////////////////////////////////////
		if(cache->diffuseblend != 1.0f)		// blend it with irrad map.. is it 0.0 or 1.0 based ??!
		{
			// ** Setup map lookup
			if( cache->oMap==miNULLTAG ){
			
				mi_error("No map saved to DB or invalid map tag !!");
				return miFALSE;
			}

			// map accessor
			Access_map  map = cache->oMap;
			if( map->is_empty() ){
				
				mi_error("Map is empty !!");
				return miFALSE;
			}

			// attach the lookup to the map
			miColor		total_irrad = {0,0,0,0};
			Map_lookup	result_lookup ( map );
			{
				// runs a lookup inside the map
				result_lookup->search ( state->point, cache->points_in_lookup );

				// retrieve irrad data
				miUint		num = 0;
				miColor		element_irrad;
				for ( ; ! result_lookup->at_end () ; result_lookup->next () )	
				{
					result_lookup->get ( cache->omap_irrad_id , element_irrad );

					total_irrad.r += element_irrad.r;
					total_irrad.g += element_irrad.g;
					total_irrad.b += element_irrad.b;
				}

				// divides by the total number of elements to get the average
				num = result_lookup->size ();
				if ( num > 0 )  {
					total_irrad.r /= ( float ) num;
					total_irrad.g /= ( float ) num;
					total_irrad.b /= ( float ) num;
				}
			}

			// blend SSS result with diffuse /////////////////////////////////////////////////////////
			result->r = total_irrad.r + cache->diffuseblend * (result->r - total_irrad.r);
			result->g = total_irrad.g + cache->diffuseblend * (result->g - total_irrad.g);
			result->b = total_irrad.b + cache->diffuseblend * (result->b - total_irrad.b);
		}

	}

	// scales the result by the color multiplier
	result->r *= cache->multiply_factor;
	result->g *= cache->multiply_factor;
	result->b *= cache->multiply_factor;
	result->a = 1.0f;
	return ( miTRUE );
}
