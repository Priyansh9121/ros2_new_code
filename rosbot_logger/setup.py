from glob import glob
from setuptools import setup
import os

package_name = "rosbot_logger"            # ← the ONLY Python package here

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],              # ← no extra names!
    data_files=[
        # 1. ament-index marker (one-line file named exactly like the package):
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),

        # 2. package manifest:
        (os.path.join("share", package_name), ["package.xml"]),

        # 3. launch files (optional):
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),

        # 4. YAML config files (optional):
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Amit Maurya",
    maintainer_email="you@example.com",
    description="Event logger node for ROSbot goals and detections",
    license="MIT",
    entry_points={
        "console_scripts": [
            "logger_node = rosbot_logger.logger_node:main",
        ],
    },
)

