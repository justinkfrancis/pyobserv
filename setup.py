from setuptools import find_packages, setup

setup(
    name="pyobserv",
    version="0.1",
    packages=find_packages(),
    url="",
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    license="MIT",
    author="Justin Francis",
    author_email="me@justinfrancis.dev",
    description="Python Observer / Ovservable design pattern with focus "
                "sensor data.",
    python_requires=">=3.6",
    install_requires=[]
)
