FROM ubuntu:16.04

# Run install prerequisite script from official git repo
RUN apt-get update && \ 
    apt-get install -y git lsb-release sudo && \
    git clone -b Copter-3.5 https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    USER=`whoami` Tools/scripts/install-prereqs-ubuntu.sh -y && \
    cd .. && \
    rm -rf ardupilot

