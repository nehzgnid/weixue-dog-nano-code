from setuptools import find_packages, setup

setup(
    name="weixue-dog-nano-code",
    version="0.1.0",
    packages=find_packages(include=["common*", "rl*", "traditional*"]),
)
