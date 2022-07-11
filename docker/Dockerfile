#
#   Copyright (c) 2020 Project CHIP Authors
#   All rights reserved.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#

from ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Locale and TZ configuration
RUN apt-get update && apt-get install -y \
  locales \
  && rm -rf /var/lib/apt/lists/* \
  && localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8
ENV LANG en_US.utf8

ENV TZ=US/Pacific
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Support for unprivileged container user management
RUN apt-get update && apt-get install -y \
  gosu \
  sudo \
  && rm -rf /var/lib/apt/lists/*

# https://github.com/project-chip/connectedhomeip/blob/1de2b73bb4123af5f184eac54d1b1d76985b4f62/docs/BUILDING.md#prerequisites
RUN apt-get update && apt-get install -y \
  git \
  gcc \
  g++ \
  python \
  pkg-config \
  libssl-dev \
  libdbus-1-dev \
  libglib2.0-dev \
  libavahi-client-dev \
  ninja-build \
  python3-venv \
  python3-dev \
  python3-pip \
  unzip \
  libgirepository1.0-dev \
  libcairo2-dev \
  libusb-1.0-0 \
  cmake \
  libreadline-dev \
  && rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /opt/entrypoint.sh

#CMD ["/opt/entrypoint.sh", "--fetch", "--configure", "--menuconfig", "--build"]
CMD ["/opt/entrypoint.sh", "--fetch", "--configure", "--build"]
