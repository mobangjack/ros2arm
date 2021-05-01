FROM ros:foxy AS toolchain

COPY ./sources.list /etc/apt/
COPY ./ros2-latest.list /etc/apt/sources.list.d/

RUN apt-get update && apt-get install -y \
    wget \
    unzip

RUN wget -q -nv https://dl.google.com/android/repository/android-ndk-r22b-linux-x86_64.zip
RUN unzip -qq android-ndk-r22b-linux-x86_64.zip

ENV ANDROID_NDK=/android-ndk-r22b
