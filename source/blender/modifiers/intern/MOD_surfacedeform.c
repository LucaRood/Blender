#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BLI_alloca.h"
#include "BLI_math.h"
#include "BLI_math_geom.h"
#include "BLI_task.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_editmesh.h"
#include "BKE_library_query.h"
#include "BKE_modifier.h"

#include "depsgraph_private.h"

#include "MEM_guardedalloc.h"

#include "MOD_util.h"

typedef struct SDefBindCalcData {
	BVHTreeFromMesh *treeData;
	DerivedMesh *tdm;
	SDefVert *verts;
	float (*vertexCos)[3];
} SDefBindCalcData;

static void initData(ModifierData *md)
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *) md;
	smd->target	= NULL;
	smd->verts	= NULL;
	smd->flags  = 0;
}

static void freeData(ModifierData *md)
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *) md;

	if (smd->verts) {
		int i;
		for (i = 0; i < smd->numverts; i++) {
			if (smd->verts[i].mode == MOD_SDEF_MODE_NGON)
				MEM_freeN(smd->verts[i].mean_val_coords);
		}
		
		MEM_freeN(smd->verts);
	}
}

static void copyData(ModifierData *md, ModifierData *target)
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *) md;
	SurfaceDeformModifierData *tsmd = (SurfaceDeformModifierData *) target;

	*tsmd = *smd;

	if (smd->verts) tsmd->verts = MEM_dupallocN(smd->verts);
}

static void foreachObjectLink(ModifierData *md, Object *ob, ObjectWalkFunc walk, void *userData)
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *) md;

	walk(userData, ob, &smd->target, IDWALK_NOP);
}

static void updateDepgraph(ModifierData *md, DagForest *forest,
                           struct Main *UNUSED(bmain),
                           struct Scene *UNUSED(scene),
                           Object *UNUSED(ob),
                           DagNode *obNode)
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *) md;

	if (smd->target) {
		DagNode *curNode = dag_get_node(forest, smd->target);

		dag_add_relation(forest, curNode, obNode,
		                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA | DAG_RL_DATA_OB | DAG_RL_OB_OB,
		                 "Surface Deform Modifier");
	}
}

static void updateDepsgraph(ModifierData *md,
                            struct Main *UNUSED(bmain),
                            struct Scene *UNUSED(scene),
                            Object *UNUSED(ob),
                            struct DepsNodeHandle *node)
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *)md;
	if (smd->target != NULL) {
		DEG_add_object_relation(node, smd->target, DEG_OB_COMP_GEOMETRY, "Surface Deform Modifier");
	}
}

static void closestEdge(int indices[2], const float point[3], const float verts[][3], int num) {
	int ind_prev, ind_curr;
	float dist, dist_min;
	
	dist_min = FLT_MAX;
	
	ind_prev = num -1;
	
	for (ind_curr = 0; ind_curr < num; ind_curr++) {
		dist = dist_squared_to_line_segment_v3(point, verts[ind_prev], verts[ind_curr]);
		
		if (dist < dist_min) {
			dist_min = dist;
			indices[0] = ind_prev;
			indices[1] = ind_curr;
		}
		
		ind_prev = ind_curr;
	}
}

static void meanValueCoordinates(float w[], const float point[3], const float verts[][3], int num) {
	float vec_curr[3], vec_prev[3], vec_tmp[3];
	float tan_prev, tan_curr, mag_curr, mag_next;
	float tot_w = 0;
	int i, ind_curr;
	
	sub_v3_v3v3(vec_tmp, verts[num - 2], point);
	sub_v3_v3v3(vec_prev, verts[num - 1], point);
	
	mag_curr = normalize_v3(vec_prev);
	normalize_v3(vec_tmp);
	
	tan_prev = tan(acos(dot_v3v3(vec_prev, vec_tmp)) / 2.0f);
	
	for (i = 0; i < num; i++) {
		sub_v3_v3v3(vec_curr, verts[i], point);
		mag_next = normalize_v3(vec_curr);
		
		tan_curr = tan(acos(dot_v3v3(vec_curr, vec_prev)) / 2.0f);
		
		ind_curr = (i == 0) ? (num - 1) : (i - 1);
		
		if (mag_curr > FLT_EPSILON) {
			w[ind_curr] = (tan_prev + tan_curr) / mag_curr;
			tot_w += w[ind_curr];
		}
		else {
			for (i = 0; i < num; i++) {
				w[i] = 0.0f;
			}
			w[ind_curr] = 1.0f;
			return;
		}
		
		mag_curr = mag_next;
		tan_prev = tan_curr;
		copy_v3_v3(vec_prev, vec_curr);
	}
	
	for (i = 0; i < num; i++) {
		w[i] /= tot_w;
		printf("Weight: %.6f\n", w[i]);
	}
}

static void bindVert(void *userdata, void *userdata_chunk, const int i, const int UNUSED(threadid)) {
	SDefBindCalcData *data = (SDefBindCalcData *) userdata;
	BVHTreeFromMesh *treeData = data->treeData;
	DerivedMesh *tdm = data->tdm;
	SDefVert *vert = data->verts + i;
	float (*vert_co)[3] = data->vertexCos + i;
	BVHTreeNearest nearest = {NULL};
	MPoly *mpoly;
	MLoop *mloop;
	MLoopTri *looptri;
	MVert *mvert;
	float v1[3], v2[3], v3[3], proj_vert_co[3];
	float proj_v1[3], proj_v2[3], proj_v3[3];
	float norm[3], co[3];
	float plane[4];
	int j;

	nearest.dist_sq = FLT_MAX;
	nearest.index = -1;
	
	BLI_bvhtree_find_nearest(treeData->tree, vert_co, &nearest, treeData->nearest_callback, treeData);

	mpoly = tdm->getPolyArray(tdm);
	mloop = tdm->getLoopArray(tdm);
	looptri = tdm->getLoopTriArray(tdm);
	mvert = tdm->getVertArray(tdm);
	
	looptri += nearest.index;
	mpoly += looptri->poly;
	
	MLoop *loop = &mloop[mpoly->loopstart];
	float (*coords)[3] = BLI_array_alloca(coords, mpoly->totloop);
	float (*proj_coords)[3] = BLI_array_alloca(proj_coords, mpoly->totloop);
	
	for (j = 0; j < mpoly->totloop; j++, loop++) {
		copy_v3_v3(coords[j], mvert[loop->v].co);
	}
	
	copy_v3_v3(v1, mvert[mloop[looptri->tri[0]].v].co);
	copy_v3_v3(v2, mvert[mloop[looptri->tri[1]].v].co);
	copy_v3_v3(v3, mvert[mloop[looptri->tri[2]].v].co);
	
	printf("\nIndex: %i\n", i);

	if (!is_poly_convex_v3(coords, mpoly->totloop)) {
		printf("Looptri bind!\n");
		vert->mode = MOD_SDEF_MODE_LOOPTRI;
	
		cent_tri_v3(co, v1, v2, v3);
		normal_tri_v3(norm, v1, v2, v3);
		
		plane_from_point_normal_v3(plane, co, norm);
		closest_to_plane_v3(proj_vert_co, plane, vert_co);
		
		interp_weights_face_v3(vert->bary_coords, v1, v2, v3, NULL, proj_vert_co);
		
		vert->verts[0] = mloop[looptri->tri[0]].v;
		vert->verts[1] = mloop[looptri->tri[1]].v;
		vert->verts[2] = mloop[looptri->tri[2]].v;
	}
	else {
		vert->poly_index = looptri->poly;
		
		normal_poly_v3(norm, coords, mpoly->totloop);
		cent_poly_v3(co, coords, mpoly->totloop);
		
		plane_from_point_normal_v3(plane, co, norm);
		closest_to_plane_v3(proj_vert_co, plane, vert_co);
		
		closest_to_plane_v3(proj_v1, plane, v1);
		closest_to_plane_v3(proj_v2, plane, v2);
		closest_to_plane_v3(proj_v3, plane, v3);
			
		for (j = 0; j < mpoly->totloop; j++) {
			closest_to_plane_v3(proj_coords[j], plane, coords[j]);
		}
		
		if (isect_point_tri_prism_v3(proj_vert_co, proj_v1, proj_v2, proj_v3)) {
			printf("Ngon bind!\n");
			vert->mode = MOD_SDEF_MODE_NGON;
			
			vert->mean_val_coords = MEM_mallocN(sizeof(*vert->mean_val_coords) * mpoly->totloop, "SDefMeanValCoords");
			
			meanValueCoordinates(vert->mean_val_coords, proj_vert_co, proj_coords, mpoly->totloop);
			
			zero_v3(proj_vert_co);
			
			for (j = 0; j < mpoly->totloop; j++) {
				madd_v3_v3fl(proj_vert_co, coords[j], vert->mean_val_coords[j]);
			}
		}
		else {
			printf("Centroid bind!\n");
			float centroid[3];
			vert->mode = MOD_SDEF_MODE_CENTROID;
			
			closestEdge(vert->verts, proj_vert_co, proj_coords, mpoly->totloop);
			
			copy_v3_v3(centroid, co);
			
			cent_tri_v3(co, coords[vert->verts[0]], coords[vert->verts[1]], centroid);
			normal_tri_v3(norm, coords[vert->verts[0]], coords[vert->verts[1]], centroid);
			
			plane_from_point_normal_v3(plane, co, norm);
			closest_to_plane_v3(proj_vert_co, plane, vert_co);
			
			interp_weights_face_v3(vert->bary_coords, coords[vert->verts[0]], coords[vert->verts[1]], centroid, NULL, proj_vert_co);
		}
	}
	
	vert->normal_dist = len_v3v3(vert_co, proj_vert_co);
	
	sub_v3_v3v3(co, vert_co, co);
	
	if (dot_v3v3(co, norm) < 0)
		vert->normal_dist *= -1;
		
	/*copy_v3_v3(v1, mvert[mloop[looptri->tri[0]].v].co);
	copy_v3_v3(v2, mvert[mloop[looptri->tri[1]].v].co);
	copy_v3_v3(v3, mvert[mloop[looptri->tri[2]].v].co);
	
	cent_tri_v3(co, v1, v2, v3);
	normal_tri_v3(norm, v1, v2, v3);
	
	plane_from_point_normal_v3(plane, co, norm);
	closest_to_plane_v3(point, plane, vert_co);
	
	interp_weights_face_v3(vert->bary_coords, v1, v2, v3, NULL, point);
	
	vert->looptri_index = nearest.index;
	vert->normal_dist = len_v3v3(vert_co, point);
	
	sub_v3_v3v3(co, vert_co, co);
	
	if (dot_v3v3(co, norm) < 0)
		vert->normal_dist *= -1;*/
}

static void surfacedeformBind(SurfaceDeformModifierData *smd, float (*vertexCos)[3],
                              int numVerts, int numTris, DerivedMesh *tdm) {
	BVHTreeFromMesh treeData = {NULL};

	/* Create a bvh-tree of the target mesh */
	bvhtree_from_mesh_looptri(&treeData, tdm, 0.0, 2, 6);
	/*if (treeData.tree == NULL) {
		OUT_OF_MEMORY(); /* TODO: Maybe replace with modifier error
		return;
	}*/
	
	smd->numverts = numVerts;
	smd->numtris = numTris;

	SDefBindCalcData data = {.treeData = &treeData, .tdm = tdm, .verts = smd->verts, .vertexCos = vertexCos};
	BLI_task_parallel_range_ex(0, numVerts, &data, NULL, 0, bindVert,
	                           numVerts > 10000, false);

	free_bvhtree_from_mesh(&treeData);
}

static void surfacedeformModifier_do(
        ModifierData *md, Object *ob, DerivedMesh *dm,
        float (*vertexCos)[3], int numVerts)
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *) md;
	DerivedMesh *tdm;
	MPoly *mpoly;
	MLoop *mloop;
	MLoopTri *looptri;
	MVert *mvert;
	SDefVert *sdvert;
	int numTris;
	int i, j;
	float v1[3], v2[3], v3[3], co[3], norm[3];

	/* Exit function if bind flag is not set (free bind data if any) */
	if (!(smd->flags & MOD_SDEF_BIND)) {
		if (smd->verts) {
			for (i = 0; i < smd->numverts; i++) {
				if (smd->verts[i].mode == MOD_SDEF_MODE_NGON)
					MEM_freeN(smd->verts[i].mean_val_coords);
			}
			MEM_freeN(smd->verts);
			smd->verts = NULL;
		}
		return;
	}

	/* Handle target mesh both in and out of edit mode */
	if (smd->target == md->scene->obedit) {
		BMEditMesh *em = BKE_editmesh_from_object(smd->target);
		tdm = em->derivedFinal;
	}
	else
		tdm = smd->target->derivedFinal;

	numTris = tdm->getNumLoopTri(tdm);
	/*mpoly = tdm->getPolyArray(tdm);
	mloop = tdm->getLoopArray(tdm);
	mvert = tdm->getVertArray(tdm);*/

	/* If none, allocate bind data and execute bind */
	if (!(smd->verts)) {
		smd->verts = MEM_mallocN(sizeof(*smd->verts) * numVerts, "SDefBindVerts");
		surfacedeformBind(smd, vertexCos, numVerts, numTris, tdm);
		
		/*for (i = 0; i < tdm->getNumPolys(tdm); i++, mpoly++) {
			MLoop *loop = &mloop[mpoly->loopstart];
			float (*coords)[3] = BLI_array_alloca(coords, mpoly->totloop);
			
			for (j = 0; j < mpoly->totloop; j++, loop++) {
				copy_v3_v3(coords[j], mvert[loop->v].co);
			}
			
			if (is_poly_convex_v3(coords, mpoly->totloop)) {
				printf("Convex!\n");
			}
			else {
				printf("Concave!\n");
			}
		}*/
	}
	
	if (smd->numverts != numVerts) {
		modifier_setError(md, "Verts changed from %d to %d", smd->numverts, numVerts);
		tdm->release(tdm);
		return;
	}
	else if (smd->numtris != numTris) {
		modifier_setError(md, "Target tris changed from %d to %d", smd->numtris, numTris);
		tdm->release(tdm);
		return;
	}

	looptri = tdm->getLoopTriArray(tdm);
	mloop = tdm->getLoopArray(tdm);
	mvert = tdm->getVertArray(tdm);
	mpoly = tdm->getPolyArray(tdm);
	sdvert = smd->verts;
	
	/* TODO: Normal offset can be calculated outside of conditional (for all modes at once) */
	for (i = 0; i < numVerts; i++, sdvert++) {
		if (sdvert->mode & MOD_SDEF_MODE_LOOPTRI) {
			copy_v3_v3(v1, mvert[sdvert->verts[0]].co);
			copy_v3_v3(v2, mvert[sdvert->verts[1]].co);
			copy_v3_v3(v3, mvert[sdvert->verts[2]].co);
		
			cent_tri_v3(co, v1, v2, v3);
			normal_tri_v3(norm, v1, v2, v3);
			
			
			/* TODO: Replace with cent_tri_v3 */
			zero_v3(vertexCos[i]);
			madd_v3_v3fl(vertexCos[i], v1, sdvert->bary_coords[0]);
			madd_v3_v3fl(vertexCos[i], v2, sdvert->bary_coords[1]);
			madd_v3_v3fl(vertexCos[i], v3, sdvert->bary_coords[2]);
			madd_v3_v3fl(vertexCos[i], norm, sdvert->normal_dist);
		}
		else {
			MPoly poly = mpoly[sdvert->poly_index];
			
			MLoop *loop = &mloop[poly.loopstart];
			float (*coords)[3] = BLI_array_alloca(coords, poly.totloop);
			
			for (j = 0; j < mpoly->totloop; j++, loop++) {
				copy_v3_v3(coords[j], mvert[loop->v].co);
			}
			
			if (sdvert->mode & MOD_SDEF_MODE_NGON) {
				zero_v3(vertexCos[i]);
				
				for (j = 0; j < poly.totloop; j++) {
					madd_v3_v3fl(vertexCos[i], coords[j], sdvert->mean_val_coords[j]);
				}
				
				normal_poly_v3(norm, coords, poly.totloop);
				
				//printf("Normal: %.6f, %.6f, %.6f\n", norm[0], norm[1], norm[2]);
				
				madd_v3_v3fl(vertexCos[i], norm, sdvert->normal_dist);
			}
			else if (sdvert->mode & MOD_SDEF_MODE_CENTROID) {
				float cent[3];
				cent_poly_v3(cent, coords, poly.totloop);
				
				loop = &mloop[poly.loopstart];
				
				copy_v3_v3(v1, mvert[loop[sdvert->verts[0]].v].co);
				copy_v3_v3(v2, mvert[loop[sdvert->verts[1]].v].co);
				
				zero_v3(vertexCos[i]);
				madd_v3_v3fl(vertexCos[i], v1, sdvert->bary_coords[0]);
				madd_v3_v3fl(vertexCos[i], v2, sdvert->bary_coords[1]);
				madd_v3_v3fl(vertexCos[i], cent, sdvert->bary_coords[2]);
				madd_v3_v3fl(vertexCos[i], norm, sdvert->normal_dist);
			}
		}
	}

	tdm->release(tdm);
}

static void deformVerts(ModifierData *md, Object *ob,
                        DerivedMesh *derivedData,
                        float (*vertexCos)[3],
                        int numVerts,
                        ModifierApplyFlag UNUSED(flag))
{
	DerivedMesh *dm = get_dm(ob, NULL, derivedData, NULL, false, false);

	surfacedeformModifier_do(md, ob, dm, vertexCos, numVerts);

	if (dm && dm != derivedData)
		dm->release(dm);
}

static void deformVertsEM(ModifierData *md, Object *ob,
                          struct BMEditMesh *UNUSED(editData),
                          DerivedMesh *derivedData,
                          float (*vertexCos)[3],
                          int numVerts)
{
	DerivedMesh *dm = get_dm(ob, NULL, derivedData, NULL, false, false);

	surfacedeformModifier_do(md, ob, dm, vertexCos, numVerts);

	if (dm && dm != derivedData)
		dm->release(dm);
}

static bool isDisabled(ModifierData *md, int UNUSED(useRenderParams))
{
	SurfaceDeformModifierData *smd = (SurfaceDeformModifierData *) md;

	return !smd->target;
}

ModifierTypeInfo modifierType_SurfaceDeform = {
	/* name */              "Surface Deform",
	/* structName */        "SurfaceDeformModifierData",
	/* structSize */        sizeof(SurfaceDeformModifierData),
	/* type */              eModifierTypeType_OnlyDeform,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_SupportsEditmode,

	/* copyData */          copyData,
	/* deformVerts */       deformVerts,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     deformVertsEM,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     NULL,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          freeData,
	/* isDisabled */        isDisabled,
	/* updateDepgraph */    updateDepgraph,
	/* updateDepsgraph */   updateDepsgraph,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */  NULL,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};
