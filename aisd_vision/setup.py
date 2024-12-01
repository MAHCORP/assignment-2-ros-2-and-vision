from setuptools import setup

package_name = 'aisd_vision'

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
    description='Vision package for hand tracking',
    license='Unspecified',
    entry_points={
        'console_scripts': [
            'image_publisher = aisd_vision.image_publisher:main',
            'hands = aisd_vision.hands:main',
        ],
    },
)
