from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.urdf import _urdf
from omni.isaac.franka.controllers import RMPFlowController
from omni.isaac.franka.tasks import FollowTarget
from omni.isaac.core.utils.nucleus import find_nucleus_server, is_file
from omni.isaac.dynamic_control import _dynamic_control
# from omni.ubb.simple_isaac_ros2.trajectory_follower import TrajectoryFollower
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims.rigid_prim import RigidPrim

import numpy as np
# import rclpy
# from rclpy.utilities import ok
from pxr import Gf, UsdPhysics

import carb
import omni
import sys

class WorldHandler(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # if not ok():
        #     rclpy.init()
        return

    def setup_scene(self):
        self.joint_names = ["panda_joint1",
                            "panda_joint2",
                            "panda_joint3",
                            "panda_joint4",
                            "panda_joint5",
                            "panda_joint6",
                            "panda_joint7",
                            "panda_finger_joint1",
                            "panda_finger_joint2",]

        self.joints = {}
        result, nucleus_server = find_nucleus_server()
        if result is False:
            carb.log_error("Could not find nucleus server with /Isaac folder, exiting")
            sys.exit()
        self.asset_path = nucleus_server + "/Isaac"
        self.franka_table_usd = self.asset_path + "/Environments/Simple_Room/Props/table_low.usd"

        self.world = self.get_world()
        self.world.scene.add_default_ground_plane()
        # Robot specific class that provides extra functionalities
        # such as having gripper and end_effector instances.

        # asset_path = nucleus_server + "/Isaac"
        # usd_path = asset_path + "/Environments/Simple_Room/simple_room.usd"
        # omni.usd.get_context().open_stage(usd_path)
        # #   ./Props/table_low.usd
        urdf_interface = _urdf.acquire_urdf_interface()
        # Set the settings in the import config
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.self_collision = True
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 10000000.0
        import_config.default_position_drive_damping = 100000.0
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.distance_scale = 100
        import_config.density = 0.0
        # Get the urdf file path
        extension_path = get_extension_path_from_name("omni.isaac.urdf")
        root_path = extension_path + "/data/urdf/robots/franka_description/robots"
        file_name = "panda_arm_hand.urdf"
        # Finally import the robot
        imported_robot = urdf_interface.parse_urdf(root_path, file_name, import_config)
        prim_path = urdf_interface.import_robot(root_path, file_name, imported_robot, import_config)

        return

    async def setup_post_load(self):
        self._world = self.get_world()
        # self._franka = self._world.scene.get_object("panda")
        # self.log_info("Num of degrees of freedom after first reset: " + str(self._franka.num_dof)) # prints 2
        # self.log_info("Joint Positions after first reset: " + str(self._franka.get_joint_positions()))

        dc = _dynamic_control.acquire_dynamic_control_interface()
        art = dc.get_articulation("/panda")
        # position, orientation = self._world.scene.get_object("fancy_cube").get_world_pose()


        if art == _dynamic_control.INVALID_HANDLE:
            print("*** '%s' is not an articulation" % "/panda")
        else:
            # Print information about articulation
            root = dc.get_articulation_root_body(art)
            print(str("Got articulation handle %d \n" % art) + str("--- Hierarchy\n"))

            body_states = dc.get_articulation_body_states(art, _dynamic_control.STATE_ALL)
            print(str("--- Body states:\n") + str(body_states) + "\n")

            dof_states = dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
            print(str("--- DOF states:\n") + str(dof_states) + "\n")

            dof_props = dc.get_articulation_dof_properties(art)
            print(str("--- DOF properties:\n") + str(dof_props) + "\n")

            for el in self.joint_names:
                try:
                        
                    dof_ptr = dc.find_articulation_dof(art, el)
                    dof_state = dc.get_dof_position(dof_ptr)
                    print(dof_state)
                    self.joints[el] = dof_ptr
                except Exception as e:
                    print(f"{el}, {dof_ptr}, {e}")
            print(self.joints)
        # if not ok():
        #     rclpy.init()
        # self.action_server = TrajectoryFollower(self.joints, "panda_hand_controller")

        return

    # async def setup_post_reset(self):
    #     self._controller.reset()
    #     await self._world.play_async()
    #     return

    # def physics_step(self, step_size):
    #     world = self.get_world()
    #     observations = world.get_observations()
    #     actions = self._controller.forward(
    #         target_end_effector_position=observations["target"]["position"],
    #         target_end_effector_orientation=observations["target"]["orientation"],
    #     )
    #     self._franka.apply_action(actions)
    #     return
