#launch Isaac Sim before any other imports
#default first two lines in any standalone application
# from omni.isaac.kit import SimulationApp
# simulation_app = SimulationApp({"headless": False}) # we can also run as headless.


from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
import omni
import omni.kit.commands
from omni.isaac.urdf import _urdf
from omni.isaac.core.utils.stage import get_stage_units
class WorldHandler(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):

        result, prim = omni.kit.commands.execute("Ros2BridgeUseSimTime", use_sim_time=False)
        # TODO: replace with ros getshare path
        omni.usd.get_context().open_stage("/home/ubb/Documents/docker_sim_comp/Isaac/ubb/Isaacdev/src/simple_arm/urdf/world.usd")
        urdf_interface = _urdf.acquire_urdf_interface()
        # Set the settings in the import config
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 10000000.0
        import_config.default_position_drive_damping = 100000.0
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.distance_scale = 100 # (cm)
        import_config.density = 0.0
        # Get the urdf file path
        root_path = "/home/ubb/Documents/docker_sim_comp/Isaac/ubb/Isaacdev/src/simple_arm/urdf/"
        file_name = "panda_isaac.urdf"
        # Finally import the robot
        imported_robot = urdf_interface.parse_urdf(root_path, file_name, import_config)
        prim_path = urdf_interface.import_robot(root_path, file_name, imported_robot, import_config, "panda")
        print(get_stage_units())
        # self.asset_path = nucleus_server + "/Isaac"
        # self.franka_table_usd = self.asset_path + "/Environments/Simple_Room/Props/table_low.usd"
        add_reference_to_stage(usd_path="/home/ubb/Documents/docker_sim_comp/Isaac/ubb/Isaacdev/src/simple_arm/urdf/table.usd",
                               prim_path="/Spawned/table")
        prim = XFormPrim(prim_path="/Spawned/table", name="table", position=np.array([70,0,-35]), orientation=np.array([0.707,0,0,0.707])) # w,x,y,z
        self.world = self.get_world()
        self.world.scene.add(prim)

        self.counters = 0
        self.objs = {"table": self.world.scene.get_object("table")}

        for x in range(2, 5):
            for y in range(-3, 4):
                self.spawn_obj("/Spawned/",[x*10,y*10,45])

        return

    def spawn_obj(self, path, position=[0, 0, 0], rotation = [1,0,0,0]):
        self.counters+=1
        name = "cube_" + str(self.counters)
        cube = self.world.scene.add(
            DynamicCuboid(
                prim_path=path+name,
                name=name,
                position=np.array(position),
                orientation = np.array(rotation),
                size=np.array([5,5,5]),
                color=np.array([0, 0, 1.0]),
            )
        )
        self.objs[name] = self.world.scene.get_object(name)

