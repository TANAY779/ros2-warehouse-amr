# Development image: ROS 2 Jazzy desktop-full (includes Gazebo Harmonic) plus clang-format
# (the editor's C++ formatter, with the repository's .clang-format) and gdb.
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
# Build time only (ARG, not ENV), so apt and dpkg can still ask questions in a dev terminal.
ARG DEBIAN_FRONTEND=noninteractive

# Ubuntu 24.04 images ship an 'ubuntu' user that owns UID/GID 1000; replace it.
# devcontainer.json adds the user to the host's render group (GID 992 by default); naming
# that GID here avoids "groups: cannot find name for group ID 992" in every shell.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel --remove ubuntu; fi \
  && groupadd --gid "${USER_GID}" "${USERNAME}" \
  && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash \
    --groups video "${USERNAME}" \
  && groupadd --gid 992 render \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
    bash-completion \
    clang-format \
    gdb \
    less \
    python3-pip \
    sudo \
  && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
  && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
  && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
# Create the workspace as the user. Otherwise Docker creates src/ as root for the bind mount
# of the repository, and `vcs import` or `ros2 pkg create` cannot write there.
RUN mkdir -p "/home/${USERNAME}/ws/src" \
  && printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    '[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash' \
    '[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' \
    >> "/home/${USERNAME}/.bashrc"

WORKDIR /home/${USERNAME}/ws
