from setuptools import find_packages, setup

package_name = 'fitomi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='wjdgus21@kyonggi.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cm = fitomi.context_manager:main',
            'tm = fitomi.task_manager:main',
            'dummy_flask = fitomi.dummy_flask:main',
            'img_display = fitomi.object_detection.img_display:main',
            'object_detect = fitomi.object_detection.object_detect:main',
            'motion = fitomi.motion:main'
        ],
    },
)
