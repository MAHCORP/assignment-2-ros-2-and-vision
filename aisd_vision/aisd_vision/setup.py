from setuptools import setup

package_name = 'aisd_motion'

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
    maintainer='Your Name',
    maintainer_email='youremail@example.com',
    description='Motion control package',
    license='Unspecified',
    entry_points={
        'console_scripts': [
            'move = aisd_motion.move:main',
        ],
    },
)
