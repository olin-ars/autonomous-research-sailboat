FROM ros:melodic-robot-bionic

RUN apt-get update && \
    apt-get -y --no-install-recommends upgrade && \
    # install a whole lot of dependencies
    apt-get install -y --no-install-recommends git curl wget sudo libgl1-mesa-glx \
        libgl1-mesa-dri mesa-utils unzip inetutils-ping bison flex build-essential g++ \
        libfl-dev libxrender1 libxtst6 libxi6 autoconf gperf tcl-dev tk-dev libgtk2.0-dev \
        software-properties-common ros-melodic-mavros* ros-melodic-joy ros-melodic-rosserial \
        ros-melodic-rosserial-arduino python-pip python-dev build-essential && \
    pip install --upgrade python pip virtualenv && \
    curl -L "https://go.microsoft.com/fwlink/?LinkID=760868" -o /tmp/code.deb && \
    apt-get -y --no-install-recommends install /tmp/code.deb && \
    # clean up temp files and caches
    apt-get -y autoremove && \
    apt-get -y clean && \
    rm -rf /tmp/* && \
    rm -rf /var/likb/apt/lists/*

# setup entrypoint and permissions
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh && \
    (echo "root:octopus" | chpasswd) && \
    groupadd oars && \
    useradd -g oars -ms /bin/bash oliner && \
    usermod -G dialout oliner && \
    (echo "oliner:password" | chpasswd)
USER oliner
WORKDIR /home/oliner

# download git repo and setup virtualenv
RUN cd /home/oliner && \
    git clone https://github.com/olin-robotic-sailing/autonomous-research-sailboat.git /home/oliner/oars-research && \
    rosdep update && \
    mkdir /home/oliner/.virtualenvs && \
    virtualenv -p python2.7 --system-site-packages /home/oliner/.virtualenvs/oars && \
    echo "export PYTHONPATH=\$PYTHONPATH:catkin_ws" >> /home/oliner/.virtualenvs/oars/bin/activate && \
    echo "alias useoars='source /home/oliner/.virtualenvs/oars/bin/activate'" >> /home/oliner/.bashrc && \
    echo "useoars" >> /home/oliner/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
