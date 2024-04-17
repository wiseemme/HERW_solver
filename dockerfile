FROM ubuntu:20.04

LABEL description="HERW Calibration Development Container" 

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends wget gnupg tar git gcc g++ make cmake ca-certificates && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libgflags-dev libtbb-dev libopencv-dev libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libfmt-dev && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/strasdat/Sophus.git && mkdir sophus-bin && cd sophus-bin && cmake ../Sophus && make && make install && cd ..