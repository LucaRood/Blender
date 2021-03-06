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
 * The Original Code is Copyright (C) 2013 Blender Foundation.
 * All rights reserved.
 *
 * Original Author: Lukas Toenne
 * Contributor(s): None Yet
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/depsgraph/intern/builder/deg_builder_nodes.h
 *  \ingroup depsgraph
 */

#pragma once

#include "intern/depsgraph_types.h"

struct Base;
struct bGPdata;
struct ListBase;
struct GHash;
struct ID;
struct FCurve;
struct Group;
struct Key;
struct Main;
struct Material;
struct MTex;
struct bNodeTree;
struct Object;
struct bPoseChannel;
struct bConstraint;
struct Scene;
struct Tex;
struct World;

struct PropertyRNA;

namespace DEG {

struct Depsgraph;
struct DepsNode;
struct RootDepsNode;
struct SubgraphDepsNode;
struct IDDepsNode;
struct TimeSourceDepsNode;
struct ComponentDepsNode;
struct OperationDepsNode;

struct DepsgraphNodeBuilder {
	DepsgraphNodeBuilder(Main *bmain, Depsgraph *graph);
	~DepsgraphNodeBuilder();

	RootDepsNode *add_root_node();
	IDDepsNode *add_id_node(ID *id);
	TimeSourceDepsNode *add_time_source(ID *id);

	ComponentDepsNode *add_component_node(ID *id,
	                                      eDepsNode_Type comp_type,
	                                      const string& comp_name = "");

	OperationDepsNode *add_operation_node(ComponentDepsNode *comp_node,
	                                      eDepsOperation_Type optype,
	                                      DepsEvalOperationCb op,
	                                      eDepsOperation_Code opcode,
	                                      const string& description = "");
	OperationDepsNode *add_operation_node(ID *id,
	                                      eDepsNode_Type comp_type,
	                                      const string& comp_name,
	                                      eDepsOperation_Type optype,
	                                      DepsEvalOperationCb op,
	                                      eDepsOperation_Code opcode,
	                                      const string& description = "");
	OperationDepsNode *add_operation_node(ID *id,
	                                      eDepsNode_Type comp_type,
	                                      eDepsOperation_Type optype,
	                                      DepsEvalOperationCb op,
	                                      eDepsOperation_Code opcode,
	                                      const string& description = "");

	bool has_operation_node(ID *id,
	                        eDepsNode_Type comp_type,
	                        const string& comp_name,
	                        eDepsOperation_Code opcode,
	                        const string& description = "");

	OperationDepsNode *find_operation_node(ID *id,
	                                       eDepsNode_Type comp_type,
	                                       const string &comp_name,
	                                       eDepsOperation_Code opcode,
	                                       const string &description = "");

	OperationDepsNode *find_operation_node(ID *id,
	                                       eDepsNode_Type comp_type,
	                                       eDepsOperation_Code opcode,
	                                       const string &description = "");

	void build_scene(Main *bmain, Scene *scene);
	SubgraphDepsNode *build_subgraph(Group *group);
	void build_group(Scene *scene, Base *base, Group *group);
	void build_object(Scene *scene, Base *base, Object *ob);
	void build_object_transform(Scene *scene, Object *ob);
	void build_object_constraints(Scene *scene, Object *ob);
	void build_pose_constraints(Object *ob, bPoseChannel *pchan);
	void build_rigidbody(Scene *scene);
	void build_particles(Scene *scene, Object *ob);
	void build_animdata(ID *id);
	OperationDepsNode *build_driver(ID *id, FCurve *fcurve);
	void build_ik_pose(Scene *scene,
	                   Object *ob,
	                   bPoseChannel *pchan,
	                   bConstraint *con);
	void build_splineik_pose(Scene *scene,
	                         Object *ob,
	                         bPoseChannel *pchan,
	                         bConstraint *con);
	void build_rig(Scene *scene, Object *ob);
	void build_proxy_rig(Object *ob);
	void build_shapekeys(Key *key);
	void build_obdata_geom(Scene *scene, Object *ob);
	void build_camera(Object *ob);
	void build_lamp(Object *ob);
	void build_nodetree(DepsNode *owner_node, bNodeTree *ntree);
	void build_material(DepsNode *owner_node, Material *ma);
	void build_texture(DepsNode *owner_node, Tex *tex);
	void build_texture_stack(DepsNode *owner_node, MTex **texture_stack);
	void build_world(World *world);
	void build_compositor(Scene *scene);
	void build_gpencil(bGPdata *gpd);

protected:
	Main *m_bmain;
	Depsgraph *m_graph;
};

}  // namespace DEG
