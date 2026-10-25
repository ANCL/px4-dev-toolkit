# syntax=docker/dockerfile:1

FROM ubuntu:24.04 AS bootstrap

ARG DEBIAN_FRONTEND=noninteractive
ARG TOOLKIT_USER=px4
ARG TOOLKIT_UID=1000
ARG TOOLKIT_GID=1000

ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

LABEL org.opencontainers.image.source="https://github.com/ANCL/px4-dev-toolkit"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Reuse an existing numeric UID/GID when the base image already owns them.
RUN existing_group="$(getent group "${TOOLKIT_GID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_group}" ] && [ "${existing_group}" != "${TOOLKIT_USER}" ]; then \
        groupmod --new-name "${TOOLKIT_USER}" "${existing_group}"; \
    elif [ -z "${existing_group}" ]; then \
        groupadd --gid "${TOOLKIT_GID}" "${TOOLKIT_USER}"; \
    fi; \
    existing_user="$(getent passwd "${TOOLKIT_UID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_user}" ] && [ "${existing_user}" != "${TOOLKIT_USER}" ]; then \
        usermod \
            --login "${TOOLKIT_USER}" \
            --home "/home/${TOOLKIT_USER}" \
            --move-home \
            "${existing_user}"; \
        usermod --gid "${TOOLKIT_USER}" "${TOOLKIT_USER}"; \
    elif [ -z "${existing_user}" ]; then \
        useradd \
            --uid "${TOOLKIT_UID}" \
            --gid "${TOOLKIT_GID}" \
            --create-home \
            --shell /bin/bash \
            "${TOOLKIT_USER}"; \
    fi; \
    usermod --append --groups dialout "${TOOLKIT_USER}"; \
    printf '%s ALL=(ALL) NOPASSWD:ALL\n' "${TOOLKIT_USER}" \
        > "/etc/sudoers.d/${TOOLKIT_USER}"; \
    chmod 0440 "/etc/sudoers.d/${TOOLKIT_USER}"

WORKDIR /opt/px4-dev-toolkit
RUN chown "${TOOLKIT_UID}:${TOOLKIT_GID}" /opt/px4-dev-toolkit

# Shared ROS 2 and source dependencies. Dependency layers precede all
# controller, analysis, documentation, and other frequently edited sources.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_ros2_jazzy.sh setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_tools.sh setup/

USER ${TOOLKIT_USER}

RUN ./setup/install_ros2_jazzy.sh desktop \
    && ./setup/install_tools.sh sitl \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

FROM bootstrap AS sitl

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/fetch_sources.sh setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/sources/sitl.repos config/sources/sitl.repos
RUN ./setup/fetch_sources.sh sitl

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_px4_dependencies.sh setup/
RUN ./setup/install_px4_dependencies.sh sitl \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_mavproxy.sh setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/mavproxy-requirements.txt config/mavproxy-requirements.txt
RUN ./setup/install_mavproxy.sh

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/runtime/sitl.env config/runtime/sitl.env
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} px4/build_sitl.sh px4/build_sitl.sh
RUN ./px4/build_sitl.sh

# The SITL image builds only its maintained ROS packages, not Vicon/mocap
# packages, which remain available through the native hardware installer.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/runtime/common.env config/runtime/common.env
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/px4_topics.def config/px4_topics.def
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/vehicles/ config/vehicles/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/trajectories.yaml config/trajectories.yaml
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/controllers/ config/controllers/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/recording/ config/recording/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} trajectory_generator/ trajectory_generator/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/build.sh ros2/build.sh
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/runtime_env.sh ros2/runtime_env.sh
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/src/offboard_controllers/ ros2/src/offboard_controllers/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/src/px4_bringup/ ros2/src/px4_bringup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/src/px4_control_common/ ros2/src/px4_control_common/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/src/px4_position_mode/ ros2/src/px4_position_mode/
RUN ./ros2/build.sh sitl

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/sequences/ config/sequences/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/tmux.conf config/tmux.conf
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/versions.env config/versions.env
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} analysis/ analysis/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/sitl tools/sitl
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/runtime/ tools/runtime/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/run_sequence tools/run_sequence
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/sequence/ tools/sequence/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/topic_rates tools/topic_rates
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/topic_rates.py tools/topic_rates.py
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/telemetry/ tools/telemetry/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/px4_analyze tools/px4_analyze
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/px4_compare tools/px4_compare
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/px4_record tools/px4_record
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/trajectory tools/trajectory
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/trajectory_lib/ tools/trajectory_lib/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/onboard_params tools/onboard_params
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/transfer/ tools/transfer/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/tmux_log tools/tmux_log
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/mavproxy_modules/ tools/mavproxy_modules/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} docker/entrypoint.sh docker/entrypoint.sh
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} LICENSE README.md ./

RUN mkdir -p bags log

# Check ULog analysis after PX4 dependency installation, which may add an
# incompatible NumPy version to the user's site-packages.
RUN .venv/onboard_analysis/bin/python3 -s -c \
    'import pyulog, numpy, matplotlib; print("ULog analysis ready")'

USER root
RUN install -m 0755 \
    /opt/px4-dev-toolkit/docker/entrypoint.sh \
    /usr/local/bin/px4-entrypoint
USER ${TOOLKIT_USER}

ENTRYPOINT ["/usr/local/bin/px4-entrypoint"]
CMD ["bash"]
