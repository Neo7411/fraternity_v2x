from setuptools import find_packages, setup

package_name = 'autoware_v2x'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xavier11',
    maintainer_email='angi.david@inf.unideb.hu',
    description='Autoware V2X module — CAM + CPM stack',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cam_read = autoware_v2x.cam.cam_read:main',
            'cam_to_obj = autoware_v2x.cam.cam_to_obj:main',
        ],
    },
)
