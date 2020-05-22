# Get the base Ubuntu image from Docker Hub
FROM gcc:latest

# Update apps on the base image
RUN apt-get -y update && apt-get install -y

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