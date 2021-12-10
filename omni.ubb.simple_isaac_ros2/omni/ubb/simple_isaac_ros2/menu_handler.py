import omni.ext
import os
from omni.isaac.examples.base_sample import BaseSampleExtension

from omni.ubb.simple_isaac_ros2 import WorldHandler
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class SimpleIsaacROS2Extension(BaseSampleExtension):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="Simple ROS2",
            submenu_name="",
            name="Cube Stacking",
            title="Cube Stacking ",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_required_hello_world.html",
            overview="This Example introduces the user on how to do cool stuff with Isaac Sim through scripting in asynchronous mode.",
            file_path=os.path.abspath(__file__),
            sample=WorldHandler(),
        )
        print("[omni.example.hello] HelloExtension startup")
