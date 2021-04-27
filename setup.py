from setuptools import setup, Extension
import os

# python setup.py sdist bdist_wheel
# twine upload dist/*

long_description = ""
with open(os.path.join(os.path.abspath(os.path.dirname(__file__)), 'README.md'), "r") as f:
    long_description = f.read()

module = Extension('virxrlcu', sources=['virxrlcu.c'])

setup(name='VirxERLU-CLib',
      version='2.0.9',
      description='C modules for VirxERLU',
      long_description=long_description,
      ext_modules=[module],
      license="MIT",
      author='VirxEC',
      author_email='virx@virxcase.dev',
      url="https://github.com/VirxEC/VirxERLU-CLib",
      python_requires='>=3.7'
      )
