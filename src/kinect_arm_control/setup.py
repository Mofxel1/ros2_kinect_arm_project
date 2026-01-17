from setuptools import find_packages, setup
import os
from glob import glob # Dosyaları taramak için gerekli kütüphane

package_name = 'kinect_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- EKLEMELER BURADA BAŞLIYOR ---
        
        # Launch dosyalarını taşı
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Config (ayar) dosyalarını taşı (YAML vb.)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        
        # Weights (Yapay zeka modelleri) klasörünü taşı
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
        
        # Description klasörü altındaki URDF klasörünün içindekileri al
        (os.path.join('share', package_name, 'description/urdf'), glob('description/urdf/*')),

        # Images (Eğer görsel varsa)
        (os.path.join('share', package_name, 'images'), glob('images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orhan',
    maintainer_email='orhanyldz987@gmail.com',
    description='Kinect ve Robot Kol Kontrol Paketi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'brain = kinect_arm_control.robot_brain:main',
        'detector_color = kinect_arm_control.detector_color:main',
        'detector_yolo = kinect_arm_control.detector_yolo:main',
    ],
},
)
