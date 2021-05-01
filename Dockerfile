FROM ros:foxy AS toolchain

RUN apt-get update && apt-get install -y \
    wget \
    unzip

RUN wget -q -nv https://dl.google.com/android/repository/android-ndk-r22b-linux-x86_64.zip
RUN unzip -qq android-ndk-r22b-linux-x86_64.zip

ENV ANDROID_NDK=/android-ndk-r22b

FROM toolchain AS manifest

WORKDIR /ros2_ws

COPY ./base.yml .
COPY ./ros2.yml .

FROM manifest AS source

RUN mkdir src
RUN vcs-import src < base.yml
RUN vcs-import src < ros2.yml

FROM source AS build

# android build configuration
ARG ANDROID_ABI=armeabi-v7a
ARG ANDROID_NATIVE_API_LEVEL=23

COPY ./generate_package_xml.py .
COPY ./_package.xml .
COPY ./build.sh .
COPY ./arch/ ./arch/

RUN export ANDROID_ABI=${ANDROID_ABI} \
    && export ANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL} \
    && ./build.sh
