#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
import rospy
import argparse
import underworlds
from physics_filter.physics_filter import PhysicsFilter

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
        PhysicsFilter(ctx, args.source_world, args.target_world, args.model_dir).run()
