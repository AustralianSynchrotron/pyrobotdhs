from setuptools import setup
import re

with open('pyrobotdhs/__init__.py') as file:
    version = re.search(r"__version__ = '(.*)'", file.read()).group(1)

setup(
    name='pyrobotdhs',
    version=version,
    packages=['pyrobotdhs'],
    install_requires=[
        'dcss',
        'aspyrobotmx',
        'click',
        'colorlog',
    ],
    entry_points={
        'console_scripts': [
            'pyrobotdhs=pyrobotdhs.cmd:run'
        ],
    },
)
