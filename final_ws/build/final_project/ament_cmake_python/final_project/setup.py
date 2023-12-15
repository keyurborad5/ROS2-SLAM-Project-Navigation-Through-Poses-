import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='final_project',
    version='0.0.0',
    packages=find_packages(
        include=('final_project', 'final_project.*')),
)
