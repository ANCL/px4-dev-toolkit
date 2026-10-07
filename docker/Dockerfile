# syntax=docker/dockerfile:1

FROM ubuntu:24.04 AS bootstrap

ARG DEBIAN_FRONTEND=noninteractive
ARG TOOLKIT_USER=px4
ARG TOOLKIT_UID=1000
ARG TOOLKIT_GID=1000

ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

LABEL org.opencontainers.image.source="https://github.com/Hosnooo/px4-dev-toolkit"

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

# Only stable inputs required to construct the shared base belong here.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_ros2_jazzy.sh setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_common_tools.sh setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/fetch_sources.sh setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/sources/common.repos config/sources/common.repos

USER ${TOOLKIT_USER}

RUN ./setup/install_ros2_jazzy.sh

RUN ./setup/install_common_tools.sh \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Common external repositories are reconstructed from the pinned manifest.
RUN ./setup/fetch_sources.sh common


FROM bootstrap AS sitl

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/sources/sitl.repos config/sources/sitl.repos

# Adds the pinned SITL-only sources and initializes PX4 nested submodules.
RUN ./setup/fetch_sources.sh sitl

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_px4_dependencies.sh setup/

RUN ./setup/install_px4_dependencies.sh sitl \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_mavproxy.sh setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/mavproxy-requirements.txt config/mavproxy-requirements.txt

RUN ./setup/install_mavproxy.sh

# PX4 compilation depends only on the SITL runtime selection and PX4 sources,
# not on controller, analysis, documentation, or Docker runtime files.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/runtime/sitl.env config/runtime/sitl.env
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} px4/build_sitl.sh px4/build_sitl.sh

RUN ./px4/build_sitl.sh

# ROS source changes may rebuild ROS, but cannot invalidate PX4 dependencies
# or the PX4 SITL build above.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} \
    config/runtime/common.env \
    config/runtime/common.env
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} \
    config/px4_topics.def \
    config/px4_topics.def
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} \
    config/vehicles/ \
    config/vehicles/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/build.sh ros2/build.sh
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/src/ ros2/src/

RUN ./ros2/build.sh

# Runtime configuration is copied after compilation so ordinary runtime
# changes cannot invalidate the ROS or platform dependency layers.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/ config/

# Runtime-only content comes after all expensive compilation steps.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} analysis/ analysis/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} docker/entrypoint.sh docker/entrypoint.sh
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} px4/ px4/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/ ros2/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/ setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/ tools/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} LICENSE README.md ./

RUN mkdir -p bags log

USER root

RUN install -m 0755 \
    /opt/px4-dev-toolkit/docker/entrypoint.sh \
    /usr/local/bin/px4-entrypoint

USER ${TOOLKIT_USER}

ENTRYPOINT ["/usr/local/bin/px4-entrypoint"]
CMD ["bash"]


FROM bootstrap AS experiment

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/sources/experiment.repos config/sources/experiment.repos

# Adds the pinned Vicon source and initializes PX4 nested submodules.
RUN ./setup/fetch_sources.sh experiment

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/install_px4_dependencies.sh setup/

RUN ./setup/install_px4_dependencies.sh experiment \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# ROS source changes remain downstream of the physical-platform dependency
# installation above.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} \
    config/runtime/common.env \
    config/runtime/common.env
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} \
    config/px4_topics.def \
    config/px4_topics.def
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} \
    config/vehicles/ \
    config/vehicles/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/build.sh ros2/build.sh
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/src/ ros2/src/

# vicon_receiver is already present from the experiment source manifest.
RUN ./ros2/build.sh

# Runtime configuration is copied after compilation so ordinary runtime
# changes cannot invalidate the ROS or platform dependency layers.
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} config/ config/

COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} analysis/ analysis/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} docker/entrypoint.sh docker/entrypoint.sh
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} px4/ px4/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} ros2/ ros2/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} setup/ setup/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} tools/ tools/
COPY --chown=${TOOLKIT_UID}:${TOOLKIT_GID} LICENSE README.md ./

RUN mkdir -p bags log

USER root

RUN install -m 0755 \
    /opt/px4-dev-toolkit/docker/entrypoint.sh \
    /usr/local/bin/px4-entrypoint

USER ${TOOLKIT_USER}

ENTRYPOINT ["/usr/local/bin/px4-entrypoint"]
CMD ["bash"]
