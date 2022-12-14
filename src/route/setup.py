from setuptools import setup

package_name = 'route'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='liu.siyao@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        ['main = route.key_joy_node:main', 'hw1 = route.hw1:main', 'hw2 = route.hw2:main', 'hw3 = route.hw3:main'],
    },
)
