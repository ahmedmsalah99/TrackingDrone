from setuptools import find_packages, setup
import glob

package_name = 'moc_video_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/videos', glob.glob('videos/*.mp4')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stark',
    maintainer_email='ahmed.msalah99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            'moc_video_publisher_node = moc_video_publisher.moc_video_publisher_node:main'
        ],
    },
)
