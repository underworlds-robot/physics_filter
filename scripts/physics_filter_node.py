#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import time
import sys
import rospy
import argparse
import underworlds
import pybullet as p
import pybullet_data
import rospy
import re
from physics_filter.srv import Pick, Release
from underworlds.helpers.transformations import *
from underworlds.helpers.geometry import *
from underworlds.types import NEW, UPDATE

EPSILON = 0.001

class PhysicsFilter(object):
    def __init__(self, ctx, source_world, target_world, model_dir, regex, simulation_step):

        self.ctx = ctx

        self.source = ctx.worlds[source_world]
        self.target = ctx.worlds[target_world]

        self.node_mapping = {self.source.scene.rootnode.id : self.target.scene.rootnode.id}
        self.underworlds_to_bullet = {}

        self.update_table = {}
        self.transforms = {}

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
        time.sleep(0.2)

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
            # reparent to root
            node.transformation = get_world_transform(scene, node)
            node.parent = scene.rootnode.id
            scene.nodes.update(node)

    def update_output_nodes(self, nodes_ids):
        nodes = []
        for node_id in nodes_ids:
            output_node = self.source.scene.nodes[node_id].copy()
            if node_id in self.node_mapping:
                output_node.id = self.node_mapping[node_id]
            else:
                self.node_mapping[node_id] = output_node.id

            if self.source.scene.nodes[node_id].properties["physics"]:
                t, q = p.getBasePositionAndOrientation(self.underworlds_to_bullet[node_id])
                t_list = list(t)
                if t_list[2]<0:
                    t_list[2]=0
                t = tuple(t_list)
                output_node.transformation = numpy.dot(translation_matrix(t), quaternion_matrix(q))

            if output_node.parent in self.node_mapping:
                output_node.parent = self.node_mapping[output_node.parent]
            if output_node.id in self.target.scene.nodes:
                if not numpy.allclose(self.target.scene.nodes[output_node.id].transformation, get_world_transform(self.target.scene, output_node)):
                    nodes.append(output_node)
            else:
                nodes.append(output_node)
        # finally we update the output world
        self.target.scene.nodes.update(nodes)
        return nodes

    def update_bullet_nodes(self, nodes_ids):
        """ This function load the urdf corresponding to the uwds node and set it in the environment
        The urdf need to have the same name than the node name
        :return:
        """
        for node_id in nodes_ids:
            node = self.source.scene.nodes[node_id]
            if node.properties["physics"] is True:
                if node.id not in self.underworlds_to_bullet:
                    self.transforms[node.id] = get_world_transform(self.source.scene, node)
                    t = translation_from_matrix(self.transforms[node.id])
                    start_position = [t[0], t[1], t[2]]
                    r = euler_from_matrix(self.transforms[node.id], 'rxyz')
                    start_orientation = p.getQuaternionFromEuler([r[0],r[1],r[2]])
                    try:
                        self.underworlds_to_bullet[node.id] = p.loadURDF(str(node.name).split("-")[0]+".urdf", start_position, start_orientation)
                    except Exception as e:
                        rospy.logwarn("[physics_filer] Exception occured : "+str(e))
                        node.properties["physics"] = False
                        self.source.scene.nodes.update(node)
                else:
                    if not numpy.allclose(self.transforms[node.id], get_world_transform(self.source.scene, node), rtol=0, atol=EPSILON):
                        self.transforms[node.id] = get_world_transform(self.source.scene, node)
                        t = translation_from_matrix(self.transforms[node.id])
                        position = [t[0], t[1], t[2]]
                        r = euler_from_matrix(self.transforms[node.id], 'rxyz')
                        orientation = p.getQuaternionFromEuler([r[0], r[1], r[2]])
                        # reset velocity of the
                        p.resetBaseVelocity(self.underworlds_to_bullet[node.id], [0.0, 0.0, 0.0])
                        p.resetBasePositionAndOrientation(self.underworlds_to_bullet[node.id], position, orientation)
                    if node.id in self.picked_ids:
                        # if node picked we apply an external force that will compensate gravity
                        # note : we need to apply this force for each step of simulation
                        p.applyExternalForce(self.underworlds_to_bullet[node.id], -1, [0, 0, 10], position, p.WORLD_FRAME)

    def filter(self, ids_to_update):
        # create the bullet model if not created and update poses in simulation
        self.update_bullet_nodes(ids_to_update)

        p.stepSimulation()  # perform the simulation step

        # then we compute the output poses of the filter
        self.update_output_nodes(ids_to_update)

    def run(self):
        ids_to_update = [node.id for node in self.source.scene.nodes if node != self.source.scene.rootnode]
        self.filter(ids_to_update)
        while not rospy.is_shutdown():
            ids_to_update = [node.id for node in self.source.scene.nodes if node != self.source.scene.rootnode]
            self.filter(ids_to_update)

    def __del__(self):
        del self.target
        p.disconnect()

if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description="Simulate the physics of the world")
    parser.add_argument("source_world", help="Underworlds world to process")
    parser.add_argument("target_world", help="Output Underworlds world")
    parser.add_argument("regex", help="The regex that math the objects simulated")
    parser.add_argument("model_dir", help="The path to URDF/Meshes directory")
    parser.add_argument("simulation_step", help="The simulation step")
    args = parser.parse_args()

    rospy.init_node("physics_filter", anonymous=True)
    with underworlds.Context("Physics Filter") as ctx:
        PhysicsFilter(ctx, args.source_world, args.target_world, args.model_dir, args.regex, float(args.simulation_step)).run()
