FROM ros:melodic-robot-bionic

ENV HOME /home/oliner
WORKDIR /home/oliner

RUN export uid=1000 gid=1000 && \
	mkdir -p /home/oliner && \
	mkdir -p /etc/sudoers.d && \
	echo "oliner:x:${uid}:${gid}:Developer,,,:/home/oliner:/bin/bash" >> /etc/passwd && \
	echo "oliner:x:${uid}:" >> /etc/group && \
	echo "oliner ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/oliner && \
	chmod 0440 /etc/sudoers.d/oliner && \
	chown ${uid}:${gid} -R /home/oliner && \
	apt-get update && \
	apt-get -y --no-install-recommends upgrade && \
	# install a whole lot of dependencies
	apt-get install -y --no-install-recommends git curl wget sudo libgl1-mesa-glx \
	libgl1-mesa-dri mesa-utils unzip inetutils-ping bison flex build-essential g++ \
	libfl-dev libxrender1 libxtst6 libxi6 autoconf gperf tcl-dev tk-dev libgtk2.0-dev \
	software-properties-common ros-melodic-mavros* ros-melodic-joy ros-melodic-rosserial \
	ros-melodic-rosserial-arduino python-pip python-dev build-essential && \
	pip install --upgrade python pip virtualenv && \
	# Install VS Code
	curl -L "https://go.microsoft.com/fwlink/?LinkID=760868" -o /tmp/code.deb && \
	apt-get -y --no-install-recommends install /tmp/code.deb && \
	# Install Emacs
	apt-get -y install emacs25 && \
	# clean up temp files and caches
	apt-get -y autoremove && \
	apt-get -y clean && \
	rm -rf /tmp/* && \
	rm -rf /var/likb/apt/lists/*

RUN sed "s/^dialout.*/&oliner/" /etc/group -i \
	&& sed "s/^root.*/&oliner/" /etc/group -i

ENV DISPLAY :1.0

# setup entrypoint and permissions
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
USER oliner

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
