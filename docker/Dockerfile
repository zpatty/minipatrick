# Get the base Ubuntu image from Docker Hub
# FROM gcc:10.1
FROM ubuntu:18.04

# Update apps on the base image
RUN apt-get -y update && apt-get install -y

RUN apt-get install -y software-properties-common

RUN add-apt-repository ppa:ubuntu-toolchain-r/test

RUN apt-get -y update && apt-get -y install gcc-10 build-essential

# Install the CMake compiler
RUN apt-get -y install cmake 

# Copy the current folder which contains C++ source code to the Docker image under /usr/src
COPY . /usr/src/dockertest1

# Specify the working directory
WORKDIR /usr/src/dockertest1

# Use Clang to compile the Test.cpp source file
RUN cmake . && make

# Run the output program from the previous step
CMD ["./Test"]