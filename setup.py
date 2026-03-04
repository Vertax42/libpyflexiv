from setuptools import setup, find_packages

setup(
    name="flexiv_bindings",
    version="0.1.0",
    description="Python bindings for Flexiv RDK real-time control",
    packages=find_packages(),
    package_data={"flexiv_bindings": ["*.so", "*.pyd"]},
    python_requires=">=3.8",
    install_requires=["pybind11"],
)
