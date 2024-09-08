from setuptools import find_packages
from setuptools import setup

package_name = 'mi_primer_paquete'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Franco Cipollone',
    author_email='franco.c@ekumenlabs.com',
    maintainer='Franco Cipollone',
    maintainer_email='franco.c@ekumenlabs.com',  # noqa: E501
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Mi primer paquete'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = mi_primer_paquete.listener:main',
            'talker = mi_primer_paquete.talker:main',
            'talker_while = mi_primer_paquete.talker_while:main',
        ],
    },
)
