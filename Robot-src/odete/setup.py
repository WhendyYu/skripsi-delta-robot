from setuptools import setup

package_name = "odete"

setup(

    name=package_name,

    version="0.0.0",

    packages=[package_name],

    data_files=[

        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),

        (
            "share/" + package_name,
            ["package.xml"],
        ),

        (
            "share/" + package_name + "/launch",
            [
                "launch/odete.launch.py",
            ],
        ),

        (
            "share/" + package_name + "/config",
            [
                "config/workspace.yaml",
                "config/detector_params.yaml",
                "config/camera_calibration.json",
            ],
        ),
    ],

    install_requires=["setuptools"],

    zip_safe=True,

    maintainer="quack",

    maintainer_email="quack@todo.todo",

    description="Object detection package",

    license="TODO",

    tests_require=["pytest"],

    entry_points={

        "console_scripts": [

            "odete = odete.odete:main",
        ],
    },
)