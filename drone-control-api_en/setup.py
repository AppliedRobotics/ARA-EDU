from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="drone_control_api",  
    version="0.1.0",
    author="DroneCam.ltd",
    author_email="dronecam@mail.ru",
    description="API package for applied robotics X dronecam EDU Drone",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/applied_robotics/drone_control_api",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    install_requires=[
        "websockets",
        "opencv-python",
        "numpy",
    ],
    extras_require={
        "dev": [
            "pytest",
            "flake8",
        ],
    },
) 