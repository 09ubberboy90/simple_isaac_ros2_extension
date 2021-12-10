import omni.ext


# Functions and vars are available to other extension as usual in python: `omni.example.hello.some_public_function(x)`
def some_public_function(x: int):
    print(f"[omni.example.hello] some_public_function was called with {x}")
    return x ** x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class HelloExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[omni.example.hello] HelloExtension startup")

    def on_shutdown(self):
        print("[omni.example.hello] HelloExtension shutdown")
