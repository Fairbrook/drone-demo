from setuptools import setup

package_name = "px4_tui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "textual"],
    zip_safe=True,
    maintainer="Kevin",
    maintainer_email="kevin@nuclea.solutions",
    description="Textual TUI front-end for px4_control.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "px4_tui = px4_tui.main:main",
        ],
    },
)
