from setuptools import setup

setup(name='coglint',
      version='1.0.5',
      packages=['ui', 'difflibparser'],
      py_modules=['coglint'],
      # generate platform specific start script
      entry_points={
        'console_scripts': [
            'coglint = coglint:main'
        ]
      },
      install_requires=[],
      url="http://en.wikipedia.org/wiki/coglint",
      download_url="https://github.com/bryantfhayes/coglint",
      keywords=["lint", "python", "c++"],
      maintainer = 'Bryant Hayes',
      maintainer_email = 'bryantfhayes@gmail.com',
      classifiers=["Programming Language :: Python",
                   "Programming Language :: Python :: 2",
                   "Programming Language :: Python :: 2.6",
                   "Programming Language :: Python :: 2.7",
                   "Programming Language :: Python :: 3",
                   "Programming Language :: Python :: 3.2",
                   "Programming Language :: Python :: 3.3",
                   "Programming Language :: Python :: 3.4",
                   "Programming Language :: Python :: 3.5",
                   "Programming Language :: C++",
                   "Development Status :: 5 - Production/Stable",
                   "Environment :: Console",
                   "Topic :: Software Development :: Quality Assurance",
                   "License :: Freely Distributable"],
      description="An automated checker to make sure a C++ file follows the Cognex C++ style guide",
      long_description=open('README.md').read())
