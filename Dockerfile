FROM ubuntu:16.04

# Add cross-toolchain to path
ENV PATH="/opt/gcc-arm-none-eabi-4_9-2015q3/bin:${PATH}"

# Run install prerequisite script from official git repo
RUN apt-get update && \ 
    apt-get install -y git lsb-release sudo && \
    git config --system url.https://github.com/.insteadOf git://github.com/ && \
    git clone -b Copter-3.5 https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    USER=`whoami` Tools/scripts/install-prereqs-ubuntu.sh -y && \
    cd .. && \
    rm -rf ardupilot

