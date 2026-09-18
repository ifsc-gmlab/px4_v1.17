from setuptools import find_packages, setup


setup(
    name="flying_car_test",
    version="0.0.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
)
