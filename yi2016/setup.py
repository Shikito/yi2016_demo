from setuptools import setup

package_name = 'yi2016'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '_utils'],
    package_dir={package_name + '_utils' : package_name+'/'+package_name+'_utils'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab-4f',
    maintainer_email='lab-4f@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_object = yi2016.target_object_service:main',
        ],
    },
)
