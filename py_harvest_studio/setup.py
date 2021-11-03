from setuptools import setup


package_name = 'py_harvest_studio'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ogumak',
    maintainer_email='kzy.basect@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tomato_detector = py_harvest_studio.tomato_detector:main',
            'tomato_detector_zero = py_harvest_studio.tomato_detector_zero:main',
            'arm_sample_publisher = py_harvest_studio.raspberry_pi.arm_sample_publisher:main',
            'sample_depth_center = py_harvest_studio.sample_depth_center:main',
        ],
    },
)
