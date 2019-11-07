From ros:melodic-robot-bionic
RUN apt-get -y clean && \
    apt-get update && apt-get -y install locales && \
	locale-gen en_US.UTF-8

ENV LANG en_US.UTF-8
RUN apt-get update && apt-get install -y \
	git	curl wget sudo libgl1-mesa-glx libgl1-mesa-dri mesa-utils unzip inetutils-ping \
	bison flex build-essential g++ libfl-dev \
	libxrender1 libxtst6 libxi6 \
	autoconf gperf \
	tcl-dev tk-dev libgtk2.0-dev software-properties-common \
	ros-melodic-mavros* ros-melodic-joy ros-melodic-rosserial ros-melodic-rosserial-arduino \
	python-pip python-dev build-essential \
	python-rosinstall python-rosinstall-generator python-wstool \
	&& pip install --upgrade python pip virtualenv && \
	curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg \
	&& install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/ && \
	sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list' \
	&& apt -y install apt-transport-https && apt update && apt -y install code && \
	apt autoremove && rm -rf /var/likb/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN echo "root:octopus" | chpasswd
RUN groupadd oars

RUN useradd -g oars -ms /bin/bash oliner && \
	usermod -G dialout oliner
USER oliner
WORKDIR /home/oliner

RUN	cd /home/oliner && \
	git clone https://github.com/olin-robotic-sailing/autonomous-research-sailboat.git /home/oliner/oars-research && \
	rosdep update && mkdir /home/oliner/.virtualenvs && \
	virtualenv -p python2.7 --system-site-packages /home/oliner/.virtualenvs/oars && \
	echo "export PYTHONPATH=\$PYTHONPATH:catkin_ws" >> /home/oliner/.virtualenvs/oars/bin/activate && \
	echo "alias useoars='source /home/oliner/.virtualenvs/oars/bin/activate'" >> /home/oliner/.bashrc \
	&& echo "useoars" >> /home/oliner/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
