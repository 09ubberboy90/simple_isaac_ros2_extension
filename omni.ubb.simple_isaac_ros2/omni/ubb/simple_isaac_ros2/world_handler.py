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

class WorldHandler(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):

        result, prim = omni.kit.commands.execute("Ros2BridgeUseSimTime", use_sim_time=False)
        # TODO: replace with ros getshare path
        omni.usd.get_context().open_stage("/home/ubb/Documents/docker_sim_comp/Isaac/ubb/Isaacdev/src/simple_move_group/urdf/world.usd")
        # self.asset_path = nucleus_server + "/Isaac"
        # self.franka_table_usd = self.asset_path + "/Environments/Simple_Room/Props/table_low.usd"
        add_reference_to_stage(usd_path="/home/ubb/Documents/docker_sim_comp/Isaac/ubb/Isaacdev/src/simple_move_group/urdf/table.usd",
                               prim_path="/Spawned/table")
        prim = XFormPrim(prim_path="/Spawned/table", name="table", position=np.array([63,0,-30]), orientation=np.array([0.707,0,0,0.707])) # w,x,y,z
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

