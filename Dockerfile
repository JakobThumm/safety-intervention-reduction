FROM python:3.8

RUN apt-get -y update \
    && apt-get install --no-install-recommends -y \
    unzip \
    libglu1-mesa-dev \
    libgl1-mesa-dev \
    libosmesa6-dev \
    sudo \
    xvfb \
    wget \
    patchelf \
    ffmpeg cmake \
    libgtest-dev \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    # Download mujoco
    && mkdir /.mujoco \
    && cd /.mujoco \
    && wget -qO- 'https://github.com/deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz' | tar -xzvf -

ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/.mujoco/mujoco210/bin"

# # Args to provide in user mode
# ARG USERNAME=robot
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID
# ## User settings
# # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     #
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME


WORKDIR /usr/

COPY . /usr/workspace/
WORKDIR /usr/workspace/
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
    tar -xf eigen-3.4.0.tar.gz && \
    rm eigen-3.4.0.tar.gz && \
    mv eigen-* eigen-3.4.0 && \
    mkdir eigen-3.4.0/build
WORKDIR /usr/workspace/eigen-3.4.0/build
RUN cmake .. && make install
ENV EIGEN3_INCLUDE_DIR="/usr/workspace/eigen-3.4.0/"

WORKDIR /usr/workspace/Safety_Gym
RUN python3 -m pip install -e .

WORKDIR /usr/workspace
RUN python3 -m pip install -r requirements.txt

COPY . /usr/workspace/
WORKDIR /usr/workspace/sara-shield
RUN mv setup_no_conda.py setup.py
RUN rm -r safety_shield/build && python3 setup.py install

RUN sudo chmod -R 777 /usr/local/lib/python3.8/site-packages/mujoco_py/generated

WORKDIR /usr/workspace
CMD ["python3", "app.py"]
