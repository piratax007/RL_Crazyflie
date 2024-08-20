from setuptools import setup


PACKAGE_NAME = 'rlcrazyflie'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml'])
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
            'CrazyflieRLControl = rlcrazyflie.CrazyflieRLControl:main',
            'control = rlcrazyflie.control:main',
            'CrazyflieAPI = rlcrazyflie.CrazyflieAPI:main',
        ],
    },
)
