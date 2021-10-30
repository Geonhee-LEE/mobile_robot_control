from setuptools import setup

setup(
    name="mobile_robot_control",
    version="0.0.1",
    description="Python library to control mobile robot",
    author="Geonhee Lee",
    author_email="gunhee6392@gmail.com",
    url='https://github.com/Geonhee-LEE/mobile_robot_control.git',
    packages=["mobile_robot_control", "mobile_robot_control.envs.simple_2d_env"],
    provides=["mobile_robot_control"],
    install_requires=["matplotlib"],
    license="GNU Lesser General Public License v3",
    classifiers=[
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Operating System :: OS Independent",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ])
