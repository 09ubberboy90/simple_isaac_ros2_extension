# simple_isaac_ros2_extension

## Installation 
### Add the extension to an NVIDIA Omniverse app and enable it

1. Add the the extension by following the steps described in [Extension Search Paths](https://docs.omniverse.nvidia.com/py/kit/docs/guide/extensions.html#extension-search-paths) or simply download and unzip the latest [release](https://github.com/Toni-SM/omni.usd.schema.add_on/releases) in one of the extension folders such as ```PATH_TO_OMNIVERSE_APP/exts```

    Git url (git+https) as extension search path: 
    
    ```
    git+https://github.com/Toni-SM/omni.usd.schema.add_on.git?branch=main&dir=exts
    ```

2. Enable the extension by following the steps described in [Extension Enabling/Disabling](https://docs.omniverse.nvidia.com/py/kit/docs/guide/extensions.html#extension-enabling-disabling)

## Switching task

The task can be switched by going into the repository and changing the branch of the extension. 

## Launching task

Go into Isaac Exemples -> Simple ROS2 -> Cube Stacking
