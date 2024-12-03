from setuptools import setup

package_name = 'aisd_motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=['aisd_motion'],
    py_modules=['aisd_motion.move'],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='alhe0026@algonquinlive.com',
    maintainer='Your Name',
    maintainer_email='alhe0026@algonquinlive.com',
    description='ROS 2 package for controlling movement based on hand gestures',
    license='TODO: License',
    entry_points={
        'console_scripts': [
            'move = aisd_motion.move:main',
        ],
    },
)
