FROM python:3.8

ARG DEBIAN_FRONTEND=noninteractive

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

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
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

### USER settings
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --create-home --no-log-init --uid $USER_UID --gid $USER_GID $USERNAME

# ARG is scoped -> has to be set as environment variable
ENV HOME_PATH=/home/$USERNAME

## Copy repo directory
COPY --chown=$USERNAME . $HOME_PATH/
WORKDIR $HOME_PATH/

####### Eigen 3 #######
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
    tar -xf eigen-3.4.0.tar.gz && \
    rm eigen-3.4.0.tar.gz && \
    mv eigen-* eigen_tmp && \
    mv eigen_tmp eigen-3.4.0 && \
    mkdir eigen-3.4.0/build
WORKDIR $HOME_PATH/eigen-3.4.0/build
RUN cmake .. && make install
ENV EIGEN3_INCLUDE_DIR="$HOME_PATH/eigen-3.4.0/"

###### Safety Gym ######
WORKDIR $HOME_PATH/Safety_Gym
RUN python3 -m pip install -e .

###### Base Repo ######
WORKDIR $HOME_PATH
RUN python3 -m pip install -r requirements.txt

###### Sara Shield ######
WORKDIR $HOME_PATH/sara-shield
RUN mv setup_no_conda.py setup.py
RUN rm -r safety_shield/build && python3 setup.py install

###### MUJOCO ######
RUN mkdir $HOME_PATH/.mujoco
WORKDIR $HOME_PATH/.mujoco
RUN curl -LJO https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz && \
    tar -xvf mujoco210-linux-x86_64.tar.gz && \
    rm mujoco210-linux-x86_64.tar.gz
#RUN mkdir /.mujoco \
#    && cd /.mujoco \
#    && wget -qO- 'https://github.com/deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz' | tar -xzvf -
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME_PATH/.mujoco/mujoco210/bin"
RUN sudo chmod -R 777 /usr/local/lib/python3.8/site-packages/mujoco_py

USER $USERNAME

WORKDIR $HOME_PATH
CMD ["python3", "app.py"]
