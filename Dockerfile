FROM ros:kinetic-ros-core-xenial
MAINTAINER Yiping Xie yipingx@kth.se
RUN apt-get update \
	&& apt-get install -y libcereal-dev libglfw3-dev libceres-dev libtinyxml2-dev libopencv-dev python-pip python-opencv && pip install ipython==5.0
RUN git clone https://github.com/nilsbore/auvlib.git
WORKDIR auvlib
RUN git submodule update --init && mkdir -p build
WORKDIR build
RUN cmake -DCMAKE_INSTALL_PREFIX=../install .. && make -j4
RUN make install

RUN echo "export PYTHONPATH=$PYTHONPATH:/auvlib/install/lib" >> ~/.bashrc && /bin/bash -c "source ~/.bashrc"  && rm -rf /var/lib/apt/lists/*



