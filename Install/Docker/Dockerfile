FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get install -y \
    build-essential \
    cmake \
    git \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    debootstrap \
    patchelf \
    xxd \
    python3-pip

RUN git clone -b 3.4.1 --single-branch https://github.com/opencv/opencv.git /opt/opencv \
    && git clone -b 3.4.1 --single-branch https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib

WORKDIR /opt/opencv/build

RUN cmake \
    -DWITH_QT=OFF \
    -DWITH_OPENGL=OFF \ 
    -DBUILD_SHARED_LIBS=ON \
    -DFORCE_VTK=ON \
    -DWITH_TBB=ON \
    -DWITH_GDAL=ON \    
    -DWITH_XINE=ON \
    -DWITH_CUDA=OFF \
    -DQT_NO_VERSION_TAGGING=ON \
    -DWITH_PNG=OFF \
    -DWITH_TIFF=OFF \
    -DWITH_WEBP=OFF \
    -DWITH_OPENJPEG=OFF \ 
    -DWITH_JASPER=OFF \
    -DWITH_OPENEXR=OFF \
    -DBUILD_TBB=ON \
    -DCMAKE_CXX_FLAGS=--std=c++17  ..


RUN make -j$(nproc)
RUN make install

RUN rm -rf /opt/opencv /opt/opencv_contrib

RUN pip3 install gdown

WORKDIR /workspace

# Daniele: To be changed in case we have a different SoftBank root image
RUN gdown https://drive.google.com/uc?id=1Xuickk8GqbFqIp_ug9Srprn3xHbtVzkc -O /

CMD ["bash"]
