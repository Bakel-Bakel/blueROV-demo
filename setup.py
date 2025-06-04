# setup.py

from setuptools import setup, find_packages

setup(
    name='GII BlueROV Libraries',
    version='0.1',
    packages=find_packages(),
    description='Custom Library for GII BlueROV',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='GII',
    install_requires=[
        'dronekit',
        'pymavlink'
    ],
    python_requires='>=3.6',
)
