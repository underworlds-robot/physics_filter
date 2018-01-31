#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import pybullet as p
import pybullet_data
import time
import rospy
import sys
import re
import underworlds
from physics_filter.msg import Pick, Release
from underworlds.helpers.transformations import *
from underworlds.helpers.geometry import *

EPSILON = 0.001

class PhysicsFilter(object):
    def __init__(self, ctx, source_world, target_world, model_dir, regex, simulation_step):

        self.ctx = ctx

        self.source = ctx.worlds[source_world]
        self.target = ctx.worlds[target_world]

        self.node_mapping = {}
        self.underworlds_to_bullet = {}

        self.update_table = {}
        self.previous_transforms = {}

        self.picked_ids = []

        self.ros_services = {"pick": rospy.Service('pick', Pick, self.handle_pick),
                             "release": rospy.Service('release', Release, self.handle_release)}

        self.physicsClient = p.connect(p.DIRECT) # initialize bullet non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        self.underworlds_to_bullet[self.source.scene.rootnode.id] = p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath(model_dir)
        p.setTimeStep(simulation_step)

        self.set_physics(regex)

    def set_physics(self, regex):
        for node in self.source.scene.nodes:
            if re.match(regex, node.name):
                node.properties["physics"] = True
                self.source.scene.nodes.update(node)

    def handle_pick(self):
        raise NotImplementedError

    def pick(self, scene, node, gripper):
        # check that the node is a mesh
        if node.type == MESH:
            # disable gravity for the node
            self.picked_ids.append(node.id)
            # disable simulation for the node
            node.properties["physics"] = False
            gripper_tf = get_world_transform(scene, gripper)
            node_tf = get_world_transform(scene, node)
            node.transform = numpy.dot(gripper_tf, node_tf.inverse())
            node.parent = gripper.id
            scene.nodes.update(node)

    def handle_release(self):
        raise NotImplementedError

    def release(self, scene, node):
        # check that the node is a mesh
        if node.type == MESH:
            # disable gravity
            self.picked_ids.remove(node.id)
            # enable simulation for the node
            node.properties["physics"] = True
            # reparent to root
            node.transformation = get_world_transform(scene, node)
            node.parent = scene.rootnode.id
            scene.nodes.update(node)

    def update_bullet_nodes_id(self, nodes_ids):
        for node_id in nodes_ids:
            node = self.source.scene.node(node_id)
            self.update_bullet_node(node)

    def update_uwds_nodes(self, nodes_ids):
        nodes = []
        for node_id in nodes_ids:
            output_node = self.source.scene.node(node_id).copy()
            if node_id in self.node_mapping[node_id]:
                output_node.id = self.node_mapping[node_id]
            else:
                self.node_mapping[node_id] = output_node.id

            if self.source.scene.node(node_id).properties["physics"] is True:
                t, q = p.getBasePositionAndOrientation(self.underworlds_to_bullet[node_id])
                output_node.transformation = numpy.dot(translation_matrix(t), quaternion_matrix(q))
            nodes.append(output_node)
        return nodes

    def update_bullet_node(self, node):
        """ This function load the urdf corresponding to the uwds node and set it in the environment
        The urdf need to have the same name than the node name
        :return:
        """
        try:
            if node.id not in self.underworlds_to_bullet:
                self.previous_transforms[node.id] = get_world_transform(self.source.scene, node)
                t = translation_from_matrix(self.previous_transforms[node.id])
                start_position = [t[0], t[1], t[2]]
                r = euler_from_matrix(self.previous_transforms[node.id], 'rxyz')
                start_orientation = p.getQuaternionFromEuler([r[0],r[1],r[2]])
                self.underworlds_to_bullet[node.id] = p.load(str(node.name).split("-")[0]+".urdf", start_position, start_orientation)
            else:
                if numpy.allclose(self.previous_transforms[node.id], get_world_transform(self.source.scene, node), rtol=0, atol=EPSILON):
                    self.previous_transforms[node.id] = get_world_transform(self.source.scene, node)
                    t = translation_from_matrix(self.start_transform[node.id])
                    position = [t[0], t[1], t[2]]
                    r = euler_from_matrix(self.start_transform[node.id], 'rxyz')
                    orientation = p.getQuaternionFromEuler([r[0], r[1], r[2]])
                    # reset velocity of the
                    p.resetBaseVelocity(self.underworlds_to_bullet[node.id])
                    p.resetBasePositionAndOrientation(self.underworlds_to_bullet[node.id], position, orientation)
                    if node.id in self.picked_ids:
                        # if node picked we apply an external force that will compensate gravity
                        # note : we need to apply this force for each step of simulation
                        p.applyExternalForce(self.underworlds_to_bullet[node.id], -1, [0, 0, 10], position, p.WORLD_FRAME)
        except Exception as e:
            rospy.logwarn("[bullet_filter] Exception occurred : "+str(e))

    def filter(self, updated_ids):
        # create the bullet model if not created and update poses in simulation
        self.update_bullet_nodes_id(updated_ids)

        p.stepSimulation()  # perform the simulation step

        # then we compute the output poses of the filter
        output_nodes = self.update_uwds_nodes(updated_ids)

        # finally we update the output world
        self.target.scene.nodes.update(output_nodes)

    def run(self):
        while not rospy.is_sutdown():
            updated_ids = self.source.scene.waitforchanges()
            self.filter(updated_ids)

    def __del__(self):
        del self.target
        p.disconnect()


