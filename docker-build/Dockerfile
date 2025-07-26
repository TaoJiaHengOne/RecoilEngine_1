FROM debian:bullseye AS mxe

ARG mxe_version=70e31e42f0584fea37aa46a74dcaa6e4eecd4304
ARG mxe_gcc=gcc13

# 添加代理设置（根据实际代理地址修改）
# 格式：http://用户名:密码@代理IP:端口（如果没有用户名密码则省略）
ENV http_proxy="http://192.168.1.15:7899" \
    https_proxy="http://192.168.1.15:7899" \
    ftp_proxy="http://192.168.1.15:7899" 
	

# 替换 Debian 11 (bullseye) 源为中科大镜像
RUN --mount=type=cache,target=/var/cache/apt \
    cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    echo "deb https://mirrors.ustc.edu.cn/debian/ bullseye main contrib non-free" > /etc/apt/sources.list && \
    echo "deb https://mirrors.ustc.edu.cn/debian/ bullseye-updates main contrib non-free" >> /etc/apt/sources.list && \
    echo "deb https://mirrors.ustc.edu.cn/debian/ bullseye-backports main contrib non-free" >> /etc/apt/sources.list && \
    echo "deb https://mirrors.ustc.edu.cn/debian-security/ bullseye-security main contrib non-free" >> /etc/apt/sources.list && \
	
# Install MXE dependencies
RUN apt-get update;apt-get install -y \
        autoconf \
        automake \
        autopoint \
        bash \
        bison \
        bzip2 \
        flex \
        g++ \
        g++-multilib \
        gettext \
        git \
        gperf \
        intltool \
        libc6-dev-i386 \
        libgdk-pixbuf2.0-dev \
        libltdl-dev \
        libssl-dev \
        libtool-bin \
        libxml-parser-perl \
        lzip \
        make \
        openssl \
        p7zip-full \
        patch \
        perl \
        python \
        python3-mako \
        ruby \
        sed \
        unzip \
        wget \
        xz-utils

# Install MXE and build cross-compile windows gcc
RUN cd /; \
    git clone https://github.com/mxe/mxe.git; \
    cd mxe; \
    git checkout $mxe_version; \
    make cc -j$(nproc) MXE_USE_CCACHE= MXE_PLUGIN_DIRS=plugins/$mxe_gcc MXE_TARGETS=x86_64-w64-mingw32.static; \
    rm -rf /mxe/pkg /mxe/src /mxe/log /mxe/.git

# Main Image
FROM ubuntu:18.04

ARG cmake_version=3.27.*
ARG ccache_version="v4.5.1"

COPY --from=mxe /mxe /mxe

ENV PATH="/mxe/usr/bin:${PATH}"

# suppress questions from apt
ENV DEBIAN_FRONTEND=noninteractive

# 第二阶段：设置阿里云镜像源
RUN --mount=type=cache,target=/var/cache/apt \
    cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ $(lsb_release -cs) main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ $(lsb_release -cs)-security main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ $(lsb_release -cs)-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ $(lsb_release -cs)-proposed main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ $(lsb_release -cs)-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    # 更新apt缓存
    apt-get update

# Install Build Dependencies (Common)
RUN apt-get update -y; \
    apt-get install --fix-missing -y \
        clang-format \
        coreutils \
        git \
        ninja-build \
        p7zip-full \
        pigz \
        software-properties-common \
        python3-pip; \
        pip3 install --upgrade pip; \
        pip3 install scikit-build; \
        pip3 install cmake==$cmake_version; \
        apt purge python3-pip -y; apt auto-remove -y

# GCC-13 PPA (linux-64)
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y; \
    apt update -y; \
    apt install -y gcc-13 g++-13

# Install Build Dependencies (linux-64)
RUN apt-get install --fix-missing -y \
        libsdl2-dev \
        libopenal-dev \
        libfreetype6-dev \
        libfontconfig1-dev

# Compile and install ccache
RUN git clone https://github.com/ccache/ccache.git /tmp/ccache/src; \
    cd /tmp/ccache/src; \
    git checkout $ccache_version; \
    mkdir -p build; \
    cd build; \
    cmake .. -DHIREDIS_FROM_INTERNET=ON -DZSTD_FROM_INTERNET=ON -DCMAKE_INSTALL_PREFIX=/tmp/ccache/bin -DCMAKE_INSTALL_SYSCONFDIR=/etc -DENABLE_DOCUMENTATION= -DENABLE_TESTING=; \
    make -j$(nproc); \
    make install; \
    cp /tmp/ccache/bin/bin/ccache /usr/bin; \
    rm -rf /tmp/ccache

# Setup ccache
ENV CCACHE_DIR=/ccache
ENV CCACHE_CONFIGPATH=/etc/ccache.conf
ENV PATH="/usr/lib/ccache:${PATH}"

COPY scripts /scripts
ENV PATH="/scripts:${PATH}"

# set primary config so no config from cache directory will be used
COPY config/ccache.conf /etc/ccache.conf

RUN mkdir -p /usr/lib/ccache; \
        /scripts/update-ccache-symlinks.sh; \
        ln -s ../../bin/ccache /usr/lib/ccache/x86_64-w64-mingw32.static-g++; \
        ln -s ../../bin/ccache /usr/lib/ccache/x86_64-w64-mingw32.static-c++; \
        ln -s ../../bin/ccache /usr/lib/ccache/x86_64-w64-mingw32.static-gcc

VOLUME /publish

ENTRYPOINT ["/bin/bash", "/scripts/entrypoint.sh"]

COPY Dockerfile /Dockerfile
