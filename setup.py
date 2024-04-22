from setuptools import find_packages, setup

package_name = 'rlcrazyflie'
rl_package = 'fausto_package'



setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, rl_package],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fausto',
    maintainer_email='piratax007@protonmail.ch',
    description='Low level control of a crazyflie using an RL policy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = rlcrazyflie.control:main', 
            'listener = rlcrazyflie.listener:main'
            # 'new_script = ' + rl_package + '.new_script:main'
        ],
    },
)
