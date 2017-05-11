from setuptools import setup

setup(
    name='telop_key_py',
    version='0.0.0',
    packages=[],
    py_modules=[
        'teleop.teleop_key_py',
    ],
    install_requires=['setuptools'],
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python node to control a robot from a keyboard.'
    ),
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'teleop_key_py = teleop.teleop_key_py:main',
        ],
    },
)
