from setuptools import setup, find_packages

setup(
    name= 'vf_fields_pkg',
    version='0.0.1',
    description='package for generating virtual forces between two robotic arms and environment',
    author='Ethan Tang',
    author_email='ethan.tang@mail.utoronto.ca',
    packages=find_packages(),
    install_requires=[
        'rospkg',
        'argparse',
        'numpy',
        'numpy-stl',
        'scipy',
        'pyyaml',
        'matplotlib'
    ],
    setup_requires=[
        'setuptools'
    ]
)