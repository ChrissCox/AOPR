from setuptools import setup

package_name = 'color_box_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'torch', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='you',
    entry_points={
        'console_scripts': [
            'sabertooth_node = color_box_nav.sabertooth_node:main',
            'vision_node = color_box_nav.vision_node:main',
        ],
    },
)
