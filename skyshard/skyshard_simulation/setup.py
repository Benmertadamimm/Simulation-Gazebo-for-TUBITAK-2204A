from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skyshard_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # resource index dosyası için package.xml'e işaret et
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]), # <--- Bu satır eskiden hataya neden oluyordu.
        
        # package.xml'i kur
        ('share/' + package_name, ['package.xml']),
        
        # Launch dosyalarının kurulumu
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # Model dosyalarının kurulumu
        (os.path.join('share', package_name, 'models', 'drone_model'), glob('models/drone_model/*')),
        (os.path.join('share', package_name, 'models', 'city_world'), glob('models/city_world/*')),
        # Rviz dosyalarının kurulumu
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Gazebo simulation setup, spawning drones, and publishing mobility state.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_manager = skyshard_simulation.scripts.spawn_manager:main',
            'marker_publisher = skyshard_simulation.scripts.marker_publisher:main',
        ],
    },
)
