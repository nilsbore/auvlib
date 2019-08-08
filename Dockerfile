FROM ros:kinetic-ros-core-xenial
MAINTAINER Yiping Xie yipingx@kth.se
RUN apt-get update \
	&& apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev build-essential python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev \
	libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev \
	libatlas-base-dev gfortran pylint \
	python2.7-dev unzip python-pip wget libcereal-dev libglfw3-dev libceres-dev libtinyxml2-dev \ 
	&& pip install ipython==5.0 \ 
    	&& rm -rf /var/lib/apt/lists/* 
RUN wget https://github.com/opencv/opencv/archive/3.3.1.zip -O opencv-3.3.1.zip && unzip opencv-3.3.1.zip && wget https://github.com/opencv/opencv_contrib/archive/3.3.1.zip -O opencv_contrib-3.3.1.zip && unzip opencv_contrib-3.3.1.zip
RUN mkdir -p opencv-3.3.1/biuld
WORKDIR opencv-3.3.1/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.3.1/modules -DOPENCV_ENABLE_NONFREE=True .. && make -j4 && make install
WORKDIR /
RUN git clone https://github.com/nilsbore/auvlib.git
WORKDIR auvlib
RUN git submodule update --init && mkdir -p build
WORKDIR libigl 
RUN git submodule update --init external/embree \
	&& git submodule update --init external/glfw 
WORKDIR ../build
RUN  cmake -DCMAKE_INSTALL_PREFIX=../install .. || exit 0 && make -j4 && make install 
RUN echo "export PYTHONPATH=$PYTHONPATH:/auvlib/install/lib" >> ~/.bashrc && /bin/bash -c "source ~/.bashrc" && rm /opencv-3.3.1.zip /opencv_contrib-3.3.1.zip && rm -rf /var/lib/apt/lists/*



