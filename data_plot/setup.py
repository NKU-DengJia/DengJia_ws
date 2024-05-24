from setuptools import find_packages, setup

package_name = 'data_plot'

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
    maintainer='dengjia',
    maintainer_email='1813400@mail.nankai.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_data_plot = data_plot.node_data_plot:main',
            'node_error_plot = data_plot.node_error_plot:main',

            'node_record_csv = data_plot.node_record_csv:main',
        ],
    },
)
