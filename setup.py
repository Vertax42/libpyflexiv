from setuptools import setup, find_packages

setup(
    name="flexiv_rt",
    version="0.2.0",
    description="Python real-time control bindings for Flexiv robots",
    packages=find_packages(),
    package_data={"flexiv_rt": ["*.so", "*.pyd"]},
    python_requires=">=3.8",
    install_requires=["pybind11"],
)
