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
 * Module:	pcloud_Shade
 * Purpose:	Point Based Shader
 *
 * Exports:
 *	pcloud_Shade
 *	pcloud_Shade_version
 *	pcloud_Shade_init
 *	pcloud_Shade_exit
 *
 */

#include "_cpc_mentalray.h"


///////////////////////////////////////////////////////////////////////////////////////////////
// * the parameter struct for the shader
struct pcloud_Shade_paras 
{
	miBoolean	verbose;

	miTag		map;	    // string-tag of the map to use
	miTag		map_attr;	// stringtag of the map field/channel
	miInteger	attr_type;	// color or float

	miScalar	mult;	    // color result multiplier
	miInteger	p_look;		// points in lookup		(opt:1)

	miColor		fg_bypass;	// fg rays don't work well with pclouds
};

// * the cache structure for the shader
struct pcloud_Shade_cache
{
	miTag tMap;		// We get the map from file, then we store in DB init
					// and access it directly from this tag in render loop
	Map_field_id	map_field_id;
	char*			map_attr_name;
	miInteger		map_attr_type;

	miScalar		multiply_factor;
	miInteger		points_in_lookup;

	miBoolean		isshaderball;
	miBoolean		disabled;
};

///////////////////////////////////////////////////////////////////////////////////////////////
// Shader Initialization
extern "C" DLLEXPORT	int	    pcloud_Shade_version(void) {return(1);}
extern "C" DLLEXPORT	void	pcloud_Shade_init (
	miState					*state,
	pcloud_Shade_paras		*paras,
	miBoolean				*inst_init_req )
{
    if ( ! paras )	{
		*inst_init_req = miTRUE; return;	// shader instance initialization
    }

	// inittialize cache
	//pcloud_Shade_cache* cache = new pcloud_Shade_cache;
	pcloud_Shade_cache* cache = (pcloud_Shade_cache*) 
								mi_mem_allocate(sizeof(pcloud_Shade_cache));

	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	*user = cache;

	// check if in hypershader
	cache->isshaderball = miFALSE;
	const miCamera * camera = state->camera;
	if( camera->focal == 1.0 || camera->x_resolution<320){
		
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

	miBoolean verbose = *mi_eval_boolean( &paras->verbose );
	if(verbose){
	mi_info ( "-----------------------------------------------" );
	mi_info ( "--- Reading point cloud" );
	mi_info ( "-----------------------------------------------" );
	}


	// evaluates the tag of the map file name
	miTag	map_tag = *mi_eval_tag ( &paras->map );
	char	*name = ( char * ) mi_db_access ( map_tag );
	mi_db_unpin ( map_tag );

	// evaluates the tag of the map field string
	miTag	map_attr_tag = *mi_eval_tag ( &paras->map_attr );
	char* map_attr_name = ( char * ) mi_db_access ( map_attr_tag );
	mi_db_unpin ( map_attr_tag );

	// access the map from file
	Access_map  map ( name );
	if(map->is_empty()){ mi_info ( "--- Map -> is empty !" );cache->tMap = miNULLTAG;return;}
	else {
		if(verbose)
		mi_info ( "--- Map -> n. of elements: %u" , map->size() );
	}

	// store the map in scene DB
	cache->tMap = map.store();

	// get a copy of the declaration
	Map_declaration	    declaration ( map );
	if(0){
		mi_info ( "--- Map -> dimensions: %u" , declaration->get_dimension () );
		mi_info ( "---" );
	}

	// cache map field to be used for lookup
	cache->map_field_id =	declaration->get_field_id ( map_attr_name );
	cache->map_attr_type = *mi_eval_integer ( &paras->attr_type );

	// cache other parameters
	cache->multiply_factor = *mi_eval_scalar ( &paras->mult );
	cache->points_in_lookup = *mi_eval_integer( &paras->p_look );



	// TEMP:DEBUG
	if(0)
	{
		Access_map_iterator	it ( map );
		int ix = 0; miVector vPos; float area; miVector vDir;
		for ( ; ! it->at_end () ; it->next () )	
		{
			it->get_position( vPos );

			if(ix%50000==0)
				mi_info( "--- Map -> point : %d, position: %.2f %.2f %.2f, dir: %f %f %f, area: %f", 
						ix, vPos.x, vPos.y, vPos.z, 
						vDir.x, vDir.y, vDir.z, area);
			ix ++;
		}
		mi_info ( "---" );
		mi_info ( "---" );
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Shader Exit
extern "C" DLLEXPORT void pcloud_Shade_exit(
	miState * state,
	pcloud_Shade_paras * paras )
{  
	if (!paras) return;

	void **user; 
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_Shade_cache* cache = static_cast< pcloud_Shade_cache* >(*user);

	if(cache->tMap) mi_db_delete(cache->tMap);	// delete data map
	mi_mem_release( *user );					// delete user cache
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Shader Implementation
extern "C" DLLEXPORT	miBoolean   pcloud_Shade (
	miColor						*result,
	miState						*state,
	struct pcloud_Shade_paras	*paras)
{
    result->r = result->g = result->b = 0.0f;
    result->a = 1.0f;


	// * Get shader cache
	void **user;
	mi_query(miQ_FUNC_USERPTR, state, 0, &user);
	pcloud_Shade_cache* cache = static_cast< pcloud_Shade_cache* >(*user);

	// exit earlier if ...  
	if(cache->isshaderball || cache->disabled) 
	{
		if(mi_is_connected(state, &paras->fg_bypass))
		{
			*result = *mi_eval_color(&paras->fg_bypass);
		}
		return miTRUE;	// shaderball render
	}

	if(	state->type==miRAY_FINALGATHER )
	{
		if(mi_is_connected(state, &paras->fg_bypass))
		{
			*result = *mi_eval_color(&paras->fg_bypass);
		}
		return miTRUE;
	}


	// ** Setup map lookup
	if(!cache->tMap) {
		mi_error("No map saved to DB or invalid map tag !!");
		return miFALSE;
	}
	Access_map  map = cache->tMap;
	if( map->is_empty() ){
	
		mi_error("Map is empty !!");
		return miFALSE;
	}

    // attach the lookup to the map
    Map_lookup	result_lookup ( map );


	// runs a lookup inside the map
	/*Map_status  status = */result_lookup->search ( state->point, cache->points_in_lookup );

	// accumulates the colors of all the elements which have been found
	miUint		num = 0;

	if(cache->map_attr_type == 0)	// color //////////////////////////////
	{
		miColor		element_color;
		for ( ; ! result_lookup->at_end () ; result_lookup->next () )	{

			// retrieves the color of the current element in the lookup.
			// It is assumed that all the maps have the same declaration, so
			// 'map_color_id' is the id of the 'color' field for all the maps
			result_lookup->get ( cache->map_field_id , element_color );

			result->r += element_color.r;
			result->g += element_color.g;
			result->b += element_color.b;
		}

	}else							// scalar ////////////////////////////
	{
		miScalar	element_scalar;
		for ( ; ! result_lookup->at_end () ; result_lookup->next () )	{

			// retrieves the color of the current element in the lookup.
			// It is assumed that all the maps have the same declaration, so
			// 'map_color_id' is the id of the 'color' field for all the maps
			result_lookup->get ( cache->map_field_id , element_scalar );

			result->r += element_scalar;
			result->g += element_scalar;
			result->b += element_scalar;
		}
	}

	// keeps track of how many elements were used in all
	num = result_lookup->size ();

	// divides by the total number of elements to get the average
	if ( num > 0 )  {
		result->r /= ( float ) num;
		result->g /= ( float ) num;
		result->b /= ( float ) num;
	}

    
	// scales the result by the color multiplier
	result->r *= cache->multiply_factor;
	result->g *= cache->multiply_factor;
	result->b *= cache->multiply_factor;
	result->a = 1.0;
	return ( miTRUE );
}