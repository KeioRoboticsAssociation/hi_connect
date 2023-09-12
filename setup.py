from setuptools import find_packages, setup

package_name = 'hi_connect'

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
    maintainer='hirobon',
    maintainer_email='hirobon1690@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'connect = hi_connect.connect:main',
            'connect_sim = hi_connect.connect_sim:main',
            'connect_seiton = hi_connect.connect_seiton:main',
            'connect_em = hi_connect.connect_em:main',
        ],
    },
)
