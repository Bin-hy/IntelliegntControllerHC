from setuptools import find_packages, setup
from glob import glob

package_name = 'lhandpro_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 添加 config 文件夹下的所有文件
        ('share/' + package_name + '/config', glob('config/*')),

        # 添加 launch 文件夹下的所有 launch 文件
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # 添加 meshes 文件夹下的所有文件（如 STL、OBJ 等）
        ('share/' + package_name + '/meshes', glob('meshes/*')),

        # 添加 urdf 文件夹下的所有文件（如 .urdf 或 .xacro 文件）
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='plf',
    maintainer_email='1399600304@qq.com',
    description='DH116 model description project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lhandpro_state_publisher = lhandpro_description.lhandpro_state_publisher:main'
        ],
    },
)
