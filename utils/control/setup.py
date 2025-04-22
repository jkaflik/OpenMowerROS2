from setuptools import setup

package_name = 'om_control'

setup(
    name=package_name,
    version='0.1.0',
    py_modules=['cli_controller'],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='OpenMower Contributor',
    author_email='',
    description='Terminal UI controller for OpenMower',
    license='',
    entry_points={
        'console_scripts': [
            'om_control = cli_controller:main',
        ],
    },
)