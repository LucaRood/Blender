/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2014 by Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Sergey Sharybin.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/intern/library_query.c
 *  \ingroup bke
 */

#include <stdlib.h>

#include "MEM_guardedalloc.h"

#include "DNA_actuator_types.h"
#include "DNA_anim_types.h"
#include "DNA_brush_types.h"
#include "DNA_camera_types.h"
#include "DNA_constraint_types.h"
#include "DNA_controller_types.h"
#include "DNA_group_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_key_types.h"
#include "DNA_lamp_types.h"
#include "DNA_lattice_types.h"
#include "DNA_linestyle_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meta_types.h"
#include "DNA_movieclip_types.h"
#include "DNA_mask_types.h"
#include "DNA_node_types.h"
#include "DNA_object_force.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"
#include "DNA_sensor_types.h"
#include "DNA_sequence_types.h"
#include "DNA_screen_types.h"
#include "DNA_speaker_types.h"
#include "DNA_sound_types.h"
#include "DNA_text_types.h"
#include "DNA_vfont_types.h"
#include "DNA_world_types.h"

#include "BLI_utildefines.h"
#include "BLI_listbase.h"
#include "BLI_ghash.h"
#include "BLI_linklist_stack.h"

#include "BKE_animsys.h"
#include "BKE_constraint.h"
#include "BKE_fcurve.h"
#include "BKE_library.h"
#include "BKE_library_query.h"
#include "BKE_main.h"
#include "BKE_modifier.h"
#include "BKE_particle.h"
#include "BKE_rigidbody.h"
#include "BKE_sca.h"
#include "BKE_sequencer.h"
#include "BKE_tracking.h"


#define FOREACH_FINALIZE _finalize
#define FOREACH_FINALIZE_VOID FOREACH_FINALIZE: (void)0

#define FOREACH_CALLBACK_INVOKE_ID_PP(_data, id_pp, cb_flag) \
	if (!((_data)->status & IDWALK_STOP)) { \
		const int _flag = (_data)->flag; \
		ID *old_id = *(id_pp); \
		const int callback_return = (_data)->callback((_data)->user_data, (_data)->self_id, id_pp, cb_flag); \
		if (_flag & IDWALK_READONLY) { \
			BLI_assert(*(id_pp) == old_id); \
		} \
		if (old_id && (_flag & IDWALK_RECURSE)) { \
			if (!BLI_gset_haskey((_data)->ids_handled, old_id)) { \
				BLI_gset_add((_data)->ids_handled, old_id); \
				if (!(callback_return & IDWALK_RET_STOP_RECURSION)) { \
					BLI_LINKSTACK_PUSH((_data)->ids_todo, old_id); \
				} \
			} \
		} \
		if (callback_return & IDWALK_RET_STOP_ITER) { \
			(_data)->status |= IDWALK_STOP; \
			goto FOREACH_FINALIZE; \
		} \
	} \
	else { \
		goto FOREACH_FINALIZE; \
	} ((void)0)

#define FOREACH_CALLBACK_INVOKE_ID(_data, id, cb_flag) \
	{ \
		CHECK_TYPE_ANY(id, ID *, void *); \
		FOREACH_CALLBACK_INVOKE_ID_PP(_data, (ID **)&(id), cb_flag); \
	} ((void)0)

#define FOREACH_CALLBACK_INVOKE(_data, id_super, cb_flag) \
	{ \
		CHECK_TYPE(&((id_super)->id), ID *); \
		FOREACH_CALLBACK_INVOKE_ID_PP(_data, (ID **)&(id_super), cb_flag); \
	} ((void)0)

/* status */
enum {
	IDWALK_STOP     = 1 << 0,
};

typedef struct LibraryForeachIDData {
	ID *self_id;
	int flag;
	LibraryIDLinkCallback callback;
	void *user_data;
	int status;

	/* To handle recursion. */
	GSet *ids_handled;  /* All IDs that are either already done, or still in ids_todo stack. */
	BLI_LINKSTACK_DECLARE(ids_todo, ID *);
} LibraryForeachIDData;

static void library_foreach_rigidbodyworldSceneLooper(
        struct RigidBodyWorld *UNUSED(rbw), ID **id_pointer, void *user_data, int cd_flag)
{
	LibraryForeachIDData *data = (LibraryForeachIDData *) user_data;
	FOREACH_CALLBACK_INVOKE_ID_PP(data, id_pointer, cd_flag);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_modifiersForeachIDLink(
        void *user_data, Object *UNUSED(object), ID **id_pointer, int cd_flag)
{
	LibraryForeachIDData *data = (LibraryForeachIDData *) user_data;
	FOREACH_CALLBACK_INVOKE_ID_PP(data, id_pointer, cd_flag);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_constraintObjectLooper(bConstraint *UNUSED(con), ID **id_pointer,
                                                   bool is_reference, void *user_data)
{
	LibraryForeachIDData *data = (LibraryForeachIDData *) user_data;
	const int cd_flag = is_reference ? IDWALK_USER : IDWALK_NOP;
	FOREACH_CALLBACK_INVOKE_ID_PP(data, id_pointer, cd_flag);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_particlesystemsObjectLooper(
        ParticleSystem *UNUSED(psys), ID **id_pointer, void *user_data, int cd_flag)
{
	LibraryForeachIDData *data = (LibraryForeachIDData *) user_data;
	FOREACH_CALLBACK_INVOKE_ID_PP(data, id_pointer, cd_flag);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_sensorsObjectLooper(
        bSensor *UNUSED(sensor), ID **id_pointer, void *user_data, int cd_flag)
{
	LibraryForeachIDData *data = (LibraryForeachIDData *) user_data;
	FOREACH_CALLBACK_INVOKE_ID_PP(data, id_pointer, cd_flag);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_controllersObjectLooper(
        bController *UNUSED(controller), ID **id_pointer, void *user_data, int cd_flag)
{
	LibraryForeachIDData *data = (LibraryForeachIDData *) user_data;
	FOREACH_CALLBACK_INVOKE_ID_PP(data, id_pointer, cd_flag);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_actuatorsObjectLooper(
        bActuator *UNUSED(actuator), ID **id_pointer, void *user_data, int cd_flag)
{
	LibraryForeachIDData *data = (LibraryForeachIDData *) user_data;
	FOREACH_CALLBACK_INVOKE_ID_PP(data, id_pointer, cd_flag);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_nla_strip(LibraryForeachIDData *data, NlaStrip *strip)
{
	NlaStrip *substrip;

	FOREACH_CALLBACK_INVOKE(data, strip->act, IDWALK_USER);

	for (substrip = strip->strips.first; substrip; substrip = substrip->next) {
		library_foreach_nla_strip(data, substrip);
	}

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_animationData(LibraryForeachIDData *data, AnimData *adt)
{
	FCurve *fcu;
	NlaTrack *nla_track;
	NlaStrip *nla_strip;

	for (fcu = adt->drivers.first; fcu; fcu = fcu->next) {
		ChannelDriver *driver = fcu->driver;
		DriverVar *dvar;

		for (dvar = driver->variables.first; dvar; dvar = dvar->next) {
			/* only used targets */
			DRIVER_TARGETS_USED_LOOPER(dvar)
			{
				FOREACH_CALLBACK_INVOKE_ID(data, dtar->id, IDWALK_NOP);
			}
			DRIVER_TARGETS_LOOPER_END
		}
	}

	FOREACH_CALLBACK_INVOKE(data, adt->action, IDWALK_USER);
	FOREACH_CALLBACK_INVOKE(data, adt->tmpact, IDWALK_USER);

	for (nla_track = adt->nla_tracks.first; nla_track; nla_track = nla_track->next) {
		for (nla_strip = nla_track->strips.first; nla_strip; nla_strip = nla_strip->next) {
			library_foreach_nla_strip(data, nla_strip);
		}
	}

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_mtex(LibraryForeachIDData *data, MTex *mtex)
{
	FOREACH_CALLBACK_INVOKE(data, mtex->object, IDWALK_NOP);
	FOREACH_CALLBACK_INVOKE(data, mtex->tex, IDWALK_USER);

	FOREACH_FINALIZE_VOID;
}

static void library_foreach_paint(LibraryForeachIDData *data, Paint *paint)
{
	FOREACH_CALLBACK_INVOKE(data, paint->brush, IDWALK_USER);
	FOREACH_CALLBACK_INVOKE(data, paint->palette, IDWALK_USER);

	FOREACH_FINALIZE_VOID;
}


/**
 * Loop over all of the ID's this datablock links to.
 *
 * \note: May be extended to be recursive in the future.
 */
void BKE_library_foreach_ID_link(ID *id, LibraryIDLinkCallback callback, void *user_data, int flag)
{
	LibraryForeachIDData data;
	int i;

	if (flag & IDWALK_RECURSE) {
		/* For now, recusion implies read-only. */
		flag |= IDWALK_READONLY;

		data.ids_handled = BLI_gset_new(BLI_ghashutil_ptrhash, BLI_ghashutil_ptrcmp, __func__);
		BLI_LINKSTACK_INIT(data.ids_todo);
	}
	else {
		data.ids_handled = NULL;
	}
	data.flag = flag;
	data.status = 0;
	data.callback = callback;
	data.user_data = user_data;

#define CALLBACK_INVOKE_ID(check_id, cb_flag) \
	FOREACH_CALLBACK_INVOKE_ID(&data, check_id, cb_flag)

#define CALLBACK_INVOKE(check_id_super, cb_flag) \
	FOREACH_CALLBACK_INVOKE(&data, check_id_super, cb_flag)

	do {
		data.self_id = id;

		AnimData *adt = BKE_animdata_from_id(id);
		if (adt) {
			library_foreach_animationData(&data, adt);
		}

		switch (GS(id->name)) {
			case ID_LI:
			{
				Library *lib = (Library *) id;
				CALLBACK_INVOKE(lib->parent, IDWALK_NOP);
				break;
			}
			case ID_SCE:
			{
				Scene *scene = (Scene *) id;
				ToolSettings *toolsett = scene->toolsettings;
				SceneRenderLayer *srl;
				Base *base;

				CALLBACK_INVOKE(scene->camera, IDWALK_NOP);
				CALLBACK_INVOKE(scene->world, IDWALK_USER);
				CALLBACK_INVOKE(scene->set, IDWALK_NOP);
				CALLBACK_INVOKE(scene->clip, IDWALK_USER);
				if (scene->nodetree) {
					/* nodetree **are owned by IDs**, treat them as mere sub-data and not real ID! */
					BKE_library_foreach_ID_link((ID *)scene->nodetree, callback, user_data, flag);
				}
				/* DO NOT handle scene->basact here, it's doubling with the loop over whole scene->base later,
				 * since basact is just a pointer to one of those items. */
				CALLBACK_INVOKE(scene->obedit, IDWALK_NOP);

				for (srl = scene->r.layers.first; srl; srl = srl->next) {
					FreestyleModuleConfig *fmc;
					FreestyleLineSet *fls;

					if (srl->mat_override) {
						CALLBACK_INVOKE(srl->mat_override, IDWALK_USER);
					}
					if (srl->light_override) {
						CALLBACK_INVOKE(srl->light_override, IDWALK_USER);
					}
					for (fmc = srl->freestyleConfig.modules.first; fmc; fmc = fmc->next) {
						if (fmc->script) {
							CALLBACK_INVOKE(fmc->script, IDWALK_NOP);
						}
					}
					for (fls = srl->freestyleConfig.linesets.first; fls; fls = fls->next) {
						if (fls->group) {
							CALLBACK_INVOKE(fls->group, IDWALK_USER);
						}
						if (fls->linestyle) {
							CALLBACK_INVOKE(fls->linestyle, IDWALK_USER);
						}
					}
				}

				if (scene->ed) {
					Sequence *seq;
					SEQP_BEGIN(scene->ed, seq)
					{
						CALLBACK_INVOKE(seq->scene, IDWALK_NOP);
						CALLBACK_INVOKE(seq->scene_camera, IDWALK_NOP);
						CALLBACK_INVOKE(seq->clip, IDWALK_USER);
						CALLBACK_INVOKE(seq->mask, IDWALK_USER);
						CALLBACK_INVOKE(seq->sound, IDWALK_USER);
						for (SequenceModifierData *smd = seq->modifiers.first; smd; smd = smd->next) {
							CALLBACK_INVOKE(smd->mask_id, IDWALK_USER);
						}
					}
					SEQ_END
				}

				CALLBACK_INVOKE(scene->gpd, IDWALK_USER);

				for (base = scene->base.first; base; base = base->next) {
					CALLBACK_INVOKE(base->object, IDWALK_USER);
				}

				if (toolsett) {
					CALLBACK_INVOKE(toolsett->skgen_template, IDWALK_NOP);

					CALLBACK_INVOKE(toolsett->particle.scene, IDWALK_NOP);
					CALLBACK_INVOKE(toolsett->particle.object, IDWALK_NOP);
					CALLBACK_INVOKE(toolsett->particle.shape_object, IDWALK_NOP);

					library_foreach_paint(&data, &toolsett->imapaint.paint);
					CALLBACK_INVOKE(toolsett->imapaint.stencil, IDWALK_USER);
					CALLBACK_INVOKE(toolsett->imapaint.clone, IDWALK_USER);
					CALLBACK_INVOKE(toolsett->imapaint.canvas, IDWALK_USER);

					if (toolsett->vpaint) {
						library_foreach_paint(&data, &toolsett->vpaint->paint);
					}
					if (toolsett->wpaint) {
						library_foreach_paint(&data, &toolsett->wpaint->paint);
					}
					if (toolsett->sculpt) {
						library_foreach_paint(&data, &toolsett->sculpt->paint);
						CALLBACK_INVOKE(toolsett->sculpt->gravity_object, IDWALK_NOP);
					}
					if (toolsett->uvsculpt) {
						library_foreach_paint(&data, &toolsett->uvsculpt->paint);
					}
				}

				if (scene->rigidbody_world) {
					BKE_rigidbody_world_id_loop(scene->rigidbody_world, library_foreach_rigidbodyworldSceneLooper, &data);
				}

				CALLBACK_INVOKE(scene->gm.dome.warptext, IDWALK_NOP);

				break;
			}

			case ID_OB:
			{
				Object *object = (Object *) id;
				ParticleSystem *psys;

				/* object data special case */
				if (object->type == OB_EMPTY) {
					/* empty can have NULL or Image */
					CALLBACK_INVOKE_ID(object->data, IDWALK_USER);
				}
				else {
					/* when set, this can't be NULL */
					if (object->data) {
						CALLBACK_INVOKE_ID(object->data, IDWALK_USER | IDWALK_NEVER_NULL);
					}
				}

				CALLBACK_INVOKE(object->parent, IDWALK_NOP);
				CALLBACK_INVOKE(object->track, IDWALK_NOP);
				/* object->proxy is refcounted, but not object->proxy_group... *sigh* */
				CALLBACK_INVOKE(object->proxy, IDWALK_USER);
				CALLBACK_INVOKE(object->proxy_group, IDWALK_NOP);
				CALLBACK_INVOKE(object->proxy_from, IDWALK_NOP);
				CALLBACK_INVOKE(object->poselib, IDWALK_USER);
				for (i = 0; i < object->totcol; i++) {
					CALLBACK_INVOKE(object->mat[i], IDWALK_USER);
				}
				CALLBACK_INVOKE(object->gpd, IDWALK_USER);
				CALLBACK_INVOKE(object->dup_group, IDWALK_USER);

				if (object->pd) {
					CALLBACK_INVOKE(object->pd->tex, IDWALK_USER);
					CALLBACK_INVOKE(object->pd->f_source, IDWALK_NOP);
				}
				/* Note that ob->effect is deprecated, so no need to handle it here. */

				if (object->pose) {
					bPoseChannel *pchan;
					for (pchan = object->pose->chanbase.first; pchan; pchan = pchan->next) {
						CALLBACK_INVOKE(pchan->custom, IDWALK_USER);
						BKE_constraints_id_loop(&pchan->constraints, library_foreach_constraintObjectLooper, &data);
					}
				}

				if (object->rigidbody_constraint) {
					CALLBACK_INVOKE(object->rigidbody_constraint->ob1, IDWALK_NOP);
					CALLBACK_INVOKE(object->rigidbody_constraint->ob2, IDWALK_NOP);
				}

				if (object->lodlevels.first) {
					LodLevel *level;
					for (level = object->lodlevels.first; level; level = level->next) {
						CALLBACK_INVOKE(level->source, IDWALK_NOP);
					}
				}

				modifiers_foreachIDLink(object, library_foreach_modifiersForeachIDLink, &data);
				BKE_constraints_id_loop(&object->constraints, library_foreach_constraintObjectLooper, &data);

				for (psys = object->particlesystem.first; psys; psys = psys->next) {
					BKE_particlesystem_id_loop(psys, library_foreach_particlesystemsObjectLooper, &data);
				}

				if (object->soft && object->soft->effector_weights) {
					CALLBACK_INVOKE(object->soft->effector_weights->group, IDWALK_NOP);
				}

				BKE_sca_sensors_id_loop(&object->sensors, library_foreach_sensorsObjectLooper, &data);
				BKE_sca_controllers_id_loop(&object->controllers, library_foreach_controllersObjectLooper, &data);
				BKE_sca_actuators_id_loop(&object->actuators, library_foreach_actuatorsObjectLooper, &data);
				break;
			}

			case ID_ME:
			{
				Mesh *mesh = (Mesh *) id;
				CALLBACK_INVOKE(mesh->texcomesh, IDWALK_USER);
				CALLBACK_INVOKE(mesh->key, IDWALK_USER);
				for (i = 0; i < mesh->totcol; i++) {
					CALLBACK_INVOKE(mesh->mat[i], IDWALK_USER);
				}
				break;
			}

			case ID_CU:
			{
				Curve *curve = (Curve *) id;
				CALLBACK_INVOKE(curve->bevobj, IDWALK_NOP);
				CALLBACK_INVOKE(curve->taperobj, IDWALK_NOP);
				CALLBACK_INVOKE(curve->textoncurve, IDWALK_NOP);
				CALLBACK_INVOKE(curve->key, IDWALK_USER);
				for (i = 0; i < curve->totcol; i++) {
					CALLBACK_INVOKE(curve->mat[i], IDWALK_USER);
				}
				CALLBACK_INVOKE(curve->vfont, IDWALK_USER);
				CALLBACK_INVOKE(curve->vfontb, IDWALK_USER);
				CALLBACK_INVOKE(curve->vfonti, IDWALK_USER);
				CALLBACK_INVOKE(curve->vfontbi, IDWALK_USER);
				break;
			}

			case ID_MB:
			{
				MetaBall *metaball = (MetaBall *) id;
				for (i = 0; i < metaball->totcol; i++) {
					CALLBACK_INVOKE(metaball->mat[i], IDWALK_USER);
				}
				break;
			}

			case ID_MA:
			{
				Material *material = (Material *) id;
				for (i = 0; i < MAX_MTEX; i++) {
					if (material->mtex[i]) {
						library_foreach_mtex(&data, material->mtex[i]);
					}
				}
				if (material->nodetree) {
					/* nodetree **are owned by IDs**, treat them as mere sub-data and not real ID! */
					BKE_library_foreach_ID_link((ID *)material->nodetree, callback, user_data, flag);
				}
				CALLBACK_INVOKE(material->group, IDWALK_USER);
				break;
			}

			case ID_TE:
			{
				Tex *texture = (Tex *) id;
				if (texture->nodetree) {
					/* nodetree **are owned by IDs**, treat them as mere sub-data and not real ID! */
					BKE_library_foreach_ID_link((ID *)texture->nodetree, callback, user_data, flag);
				}
				CALLBACK_INVOKE(texture->ima, IDWALK_USER);
				if (texture->env) {
					CALLBACK_INVOKE(texture->env->object, IDWALK_NOP);
					CALLBACK_INVOKE(texture->env->ima, IDWALK_USER);
				}
				if (texture->pd)
					CALLBACK_INVOKE(texture->pd->object, IDWALK_NOP);
				if (texture->vd)
					CALLBACK_INVOKE(texture->vd->object, IDWALK_NOP);
				if (texture->ot)
					CALLBACK_INVOKE(texture->ot->object, IDWALK_NOP);
				break;
			}

			case ID_LT:
			{
				Lattice *lattice = (Lattice *) id;
				CALLBACK_INVOKE(lattice->key, IDWALK_USER);
				break;
			}

			case ID_LA:
			{
				Lamp *lamp = (Lamp *) id;
				for (i = 0; i < MAX_MTEX; i++) {
					if (lamp->mtex[i]) {
						library_foreach_mtex(&data, lamp->mtex[i]);
					}
				}
				if (lamp->nodetree) {
					/* nodetree **are owned by IDs**, treat them as mere sub-data and not real ID! */
					BKE_library_foreach_ID_link((ID *)lamp->nodetree, callback, user_data, flag);
				}
				break;
			}

			case ID_CA:
			{
				Camera *camera = (Camera *) id;
				CALLBACK_INVOKE(camera->dof_ob, IDWALK_NOP);
				break;
			}

			case ID_KE:
			{
				Key *key = (Key *) id;
				CALLBACK_INVOKE_ID(key->from, IDWALK_NOP);
				break;
			}

			case ID_SCR:
			{
				bScreen *screen = (bScreen *) id;
				CALLBACK_INVOKE(screen->scene, IDWALK_USER_ONE);
				break;
			}

			case ID_WO:
			{
				World *world = (World *) id;
				for (i = 0; i < MAX_MTEX; i++) {
					if (world->mtex[i]) {
						library_foreach_mtex(&data, world->mtex[i]);
					}
				}
				if (world->nodetree) {
					/* nodetree **are owned by IDs**, treat them as mere sub-data and not real ID! */
					BKE_library_foreach_ID_link((ID *)world->nodetree, callback, user_data, flag);
				}
				break;
			}

			case ID_SPK:
			{
				Speaker *speaker = (Speaker *) id;
				CALLBACK_INVOKE(speaker->sound, IDWALK_USER);
				break;
			}

			case ID_GR:
			{
				Group *group = (Group *) id;
				GroupObject *gob;
				for (gob = group->gobject.first; gob; gob = gob->next) {
					CALLBACK_INVOKE(gob->ob, IDWALK_USER_ONE);
				}
				break;
			}

			case ID_NT:
			{
				bNodeTree *ntree = (bNodeTree *) id;
				bNode *node;
				CALLBACK_INVOKE(ntree->gpd, IDWALK_USER);
				for (node = ntree->nodes.first; node; node = node->next) {
					CALLBACK_INVOKE_ID(node->id, IDWALK_USER);
				}
				break;
			}

			case ID_BR:
			{
				Brush *brush = (Brush *) id;
				CALLBACK_INVOKE(brush->toggle_brush, IDWALK_NOP);
				CALLBACK_INVOKE(brush->clone.image, IDWALK_NOP);
				CALLBACK_INVOKE(brush->paint_curve, IDWALK_USER);
				library_foreach_mtex(&data, &brush->mtex);
				library_foreach_mtex(&data, &brush->mask_mtex);
				break;
			}

			case ID_PA:
			{
				ParticleSettings *psett = (ParticleSettings *) id;
				CALLBACK_INVOKE(psett->dup_group, IDWALK_NOP);
				CALLBACK_INVOKE(psett->dup_ob, IDWALK_NOP);
				CALLBACK_INVOKE(psett->bb_ob, IDWALK_NOP);

				for (i = 0; i < MAX_MTEX; i++) {
					if (psett->mtex[i]) {
						library_foreach_mtex(&data, psett->mtex[i]);
					}
				}

				if (psett->effector_weights) {
					CALLBACK_INVOKE(psett->effector_weights->group, IDWALK_NOP);
				}

				if (psett->pd) {
					CALLBACK_INVOKE(psett->pd->tex, IDWALK_USER);
					CALLBACK_INVOKE(psett->pd->f_source, IDWALK_NOP);
				}
				if (psett->pd2) {
					CALLBACK_INVOKE(psett->pd2->tex, IDWALK_USER);
					CALLBACK_INVOKE(psett->pd2->f_source, IDWALK_NOP);
				}

				if (psett->boids) {
					BoidState *state;
					BoidRule *rule;

					for (state = psett->boids->states.first; state; state = state->next) {
						for (rule = state->rules.first; rule; rule = rule->next) {
							if (rule->type == eBoidRuleType_Avoid) {
								BoidRuleGoalAvoid *gabr = (BoidRuleGoalAvoid *)rule;
								CALLBACK_INVOKE(gabr->ob, IDWALK_NOP);
							}
							else if (rule->type == eBoidRuleType_FollowLeader) {
								BoidRuleFollowLeader *flbr = (BoidRuleFollowLeader *)rule;
								CALLBACK_INVOKE(flbr->ob, IDWALK_NOP);
							}
						}
					}
				}
				break;
			}

			case ID_MC:
			{
				MovieClip *clip = (MovieClip *) id;
				MovieTracking *tracking = &clip->tracking;
				MovieTrackingObject *object;
				MovieTrackingTrack *track;
				MovieTrackingPlaneTrack *plane_track;

				CALLBACK_INVOKE(clip->gpd, IDWALK_USER);

				for (track = tracking->tracks.first; track; track = track->next) {
					CALLBACK_INVOKE(track->gpd, IDWALK_USER);
				}
				for (object = tracking->objects.first; object; object = object->next) {
					for (track = object->tracks.first; track; track = track->next) {
						CALLBACK_INVOKE(track->gpd, IDWALK_USER);
					}
				}

				for (plane_track = tracking->plane_tracks.first; plane_track; plane_track = plane_track->next) {
					CALLBACK_INVOKE(plane_track->image, IDWALK_USER);
				}
				break;
			}

			case ID_MSK:
			{
				Mask *mask = (Mask *) id;
				MaskLayer *mask_layer;
				for (mask_layer = mask->masklayers.first; mask_layer; mask_layer = mask_layer->next) {
					MaskSpline *mask_spline;

					for (mask_spline = mask_layer->splines.first; mask_spline; mask_spline = mask_spline->next) {
						for (i = 0; i < mask_spline->tot_point; i++) {
							MaskSplinePoint *point = &mask_spline->points[i];
							CALLBACK_INVOKE_ID(point->parent.id, IDWALK_USER);
						}
					}
				}
				break;
			}

			case ID_LS:
			{
				FreestyleLineStyle *linestyle = (FreestyleLineStyle *) id;
				LineStyleModifier *lsm;
				for (i = 0; i < MAX_MTEX; i++) {
					if (linestyle->mtex[i]) {
						library_foreach_mtex(&data, linestyle->mtex[i]);
					}
				}
				if (linestyle->nodetree) {
					/* nodetree **are owned by IDs**, treat them as mere sub-data and not real ID! */
					BKE_library_foreach_ID_link((ID *)linestyle->nodetree, callback, user_data, flag);
				}

				for (lsm = linestyle->color_modifiers.first; lsm; lsm = lsm->next) {
					if (lsm->type == LS_MODIFIER_DISTANCE_FROM_OBJECT) {
						LineStyleColorModifier_DistanceFromObject *p = (LineStyleColorModifier_DistanceFromObject *)lsm;
						if (p->target) {
							CALLBACK_INVOKE(p->target, IDWALK_NOP);
						}
					}
				}
				for (lsm = linestyle->alpha_modifiers.first; lsm; lsm = lsm->next) {
					if (lsm->type == LS_MODIFIER_DISTANCE_FROM_OBJECT) {
						LineStyleAlphaModifier_DistanceFromObject *p = (LineStyleAlphaModifier_DistanceFromObject *)lsm;
						if (p->target) {
							CALLBACK_INVOKE(p->target, IDWALK_NOP);
						}
					}
				}
				for (lsm = linestyle->thickness_modifiers.first; lsm; lsm = lsm->next) {
					if (lsm->type == LS_MODIFIER_DISTANCE_FROM_OBJECT) {
						LineStyleThicknessModifier_DistanceFromObject *p = (LineStyleThicknessModifier_DistanceFromObject *)lsm;
						if (p->target) {
							CALLBACK_INVOKE(p->target, IDWALK_NOP);
						}
					}
				}
				break;
			}
		}
	} while ((id = (flag & IDWALK_RECURSE) ? BLI_LINKSTACK_POP(data.ids_todo) : NULL));

FOREACH_FINALIZE:
	if (data.ids_handled) {
		BLI_gset_free(data.ids_handled, NULL);
		BLI_LINKSTACK_FREE(data.ids_todo);
	}

#undef CALLBACK_INVOKE_ID
#undef CALLBACK_INVOKE
}

#undef FOREACH_CALLBACK_INVOKE_ID
#undef FOREACH_CALLBACK_INVOKE

/**
 * re-usable function, use when replacing ID's
 */
void BKE_library_update_ID_link_user(ID *id_dst, ID *id_src, const int cd_flag)
{
	if (cd_flag & IDWALK_USER) {
		id_us_min(id_src);
		id_us_plus(id_dst);
	}
	else if (cd_flag & IDWALK_USER_ONE) {
		id_us_ensure_real(id_dst);
	}
}

/* ***** ID users iterator. ***** */
typedef struct IDUsersIter {
	ID *id;

	ListBase *lb_array[MAX_LIBARRAY];
	int lb_idx;

	ID *curr_id;
	int count;  /* Set by callback. */
} IDUsersIter;

static int foreach_libblock_id_users_callback(void *user_data, ID *UNUSED(self_id), ID **id_p, int cb_flag)
{
	IDUsersIter *iter = user_data;

	if (*id_p && (*id_p == iter->id)) {
#if 0
		printf("%s uses %s (refcounted: %d, userone: %d, used_one: %d, used_one_active: %d)\n",
		       iter->curr_id->name, iter->id->name, (cb_flag & IDWALK_USER) ? 1 : 0, (cb_flag & IDWALK_USER_ONE) ? 1 : 0,
		       (iter->id->tag & LIB_TAG_EXTRAUSER) ? 1 : 0, (iter->id->tag & LIB_TAG_EXTRAUSER_SET) ? 1 : 0);
#else
		UNUSED_VARS(cb_flag);
#endif
		iter->count++;
	}

	return IDWALK_RET_NOP;
}

/**
 * Return the number of times given \a id_user uses/references \a id_used.
 *
 * \note This only checks for pointer references of an ID, shallow usages (like e.g. by RNA paths, as done
 *       for FCurves) are not detected at all.
 *
 * \param id_user the ID which is supposed to use (reference) \a id_used.
 * \param id_used the ID which is supposed to be used (referenced) by \a id_user.
 * \return the number of direct usages/references of \a id_used by \a id_user.
 */
int BKE_library_ID_use_ID(ID *id_user, ID *id_used)
{
	IDUsersIter iter;

	/* We do not care about iter.lb_array/lb_idx here... */
	iter.id = id_used;
	iter.curr_id = id_user;
	iter.count = 0;

	BKE_library_foreach_ID_link(iter.curr_id, foreach_libblock_id_users_callback, (void *)&iter, IDWALK_NOP);

	return iter.count;
}


static int foreach_libblock_check_indirect_usage_callback(
        void *user_data, ID *UNUSED(id_self), ID **id_p, int UNUSED(cb_flag))
{
	IDUsersIter *iter = user_data;

	if (*id_p && (*id_p == iter->id)) {
		iter->count++;
		return IDWALK_RET_STOP_ITER;
	}

	return IDWALK_RET_NOP;
}

/**
 * Check wether given ID is used indirectly (i.e. by another linked ID).
 */
bool BKE_library_ID_is_indirectly_used(Main *bmain, void *idv)
{
	IDUsersIter iter;
	ListBase *lb_array[MAX_LIBARRAY];
	int i = set_listbasepointers(bmain, lb_array);

	iter.id = idv;
	iter.count = 0;
	while (i--) {
		ID *id_curr = lb_array[i]->first;

		for (; id_curr; id_curr = id_curr->next) {
			if (!id_curr->lib) {
				continue;
			}

			iter.curr_id = id_curr;
			BKE_library_foreach_ID_link(
			            id_curr, foreach_libblock_check_indirect_usage_callback, &iter, IDWALK_NOP);

			if (iter.count) {
				break;
			}
		}
		if (iter.count) {
			break;
		}
	}

	return (iter.count != 0);
}

