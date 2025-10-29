from setuptools import find_packages, setup

package_name = 'python_test_pkg'

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
    maintainer='fantasywilly',
    maintainer_email='bc697522h04@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_pub      = python_test_pkg.hello_pub_node:main',  
            'hello_sub      = python_test_pkg.hello_sub_node:main',
            'student_pub    = python_test_pkg.student_pub_node:main',
            'student_sub    = python_test_pkg.student_sub_node:main',  
            'class_pub      = python_test_pkg.class_pub_node:main', 
            'class_sub      = python_test_pkg.class_sub_node:main', 
        ],
    },
)
