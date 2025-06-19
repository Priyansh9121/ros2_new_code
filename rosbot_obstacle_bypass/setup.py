# rosbot_obstacle_bypass/setup.py
from glob import glob
from setuptools import setup
import os

package_name = "rosbot_obstacle_bypass"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],            # Python modules
    data_files=[
        # ① one (and only one) marker entry — let colcon handle duplicates
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),

        # ② install package.xml so the warning disappears
        (f"share/{package_name}", ["package.xml"]),

        # ③ launch and parameter files
        (f"share/{package_name}/launch", ["launch/bypass_node.launch.py"]),
        (f"share/{package_name}/config", ["config/params.yaml"]),
    ],

    zip_safe=True,
    maintainer="Amit Maurya",
    maintainer_email="you@example.com",
    description="ROSbot extension for obstacle bypass",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bypass_node = rosbot_obstacle_bypass.bypass_node:main",
        ],
    },
)

