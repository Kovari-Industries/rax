# syntax=docker/dockerfile:1
# The HumanoidArena simulator image: Isaac Sim 6.1 with Isaac Lab (patched for 6.1), cuRobo and the
# packages HumanoidArena's SONIC action provider imports. Build it with scripts/build_humanoid_image.sh.
# The HumanoidArena checkout and its assets are mounted at run time (scripts/humanoid_python.sh), not
# copied: the harness checks the pinned commit and asset hashes itself.
ARG ISAACSIM_IMAGE
FROM ${ISAACSIM_IMAGE}
ARG ISAACLAB_URL
ARG ISAACLAB_COMMIT
ARG CUROBO_URL
ARG CUROBO_COMMIT
USER root
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    OMNI_KIT_ALLOW_ROOT=1 TERM=xterm-256color PIP_USER=0 PYTHONNOUSERSITE=1 PIP_NO_CACHE_DIR=1 \
    FORCE_CUDA=1 TORCH_CUDA_ARCH_LIST="8.0;12.0+PTX" CUDA_HOME=/usr/local/cuda-12.8 \
    PATH=/usr/local/cuda-12.8/bin:${PATH}
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build pkg-config git git-lfs wget curl ca-certificates gnupg \
        ffmpeg netcat-openbsd libglu1-mesa libglib2.0-0 libxrender1 libsm6 libice6 \
        libxrandr2 libxi6 libxcursor1 libxinerama1 vulkan-tools \
    && git lfs install && rm -rf /var/lib/apt/lists/*
# cuRobo compiles its CUDA kernels against this toolkit.
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb && rm -f cuda-keyring_1.1-1_all.deb \
    && apt-get update && apt-get install -y --no-install-recommends cuda-toolkit-12-8 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN /isaac-sim/python.sh -m pip install --upgrade "pip" "setuptools<82" wheel setuptools_scm ninja \
    && /isaac-sim/python.sh -m pip install torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 \
        --index-url https://download.pytorch.org/whl/cu128
COPY constraints.txt /opt/isaacsim61_constraints.txt
RUN /isaac-sim/python.sh -m pip install -c /opt/isaacsim61_constraints.txt "warp-lang==1.16.0" \
        onnx "prettytable==3.3.0" toml hidapi "gymnasium==1.2.1" trimesh "pyglet<2" \
        "transformers==4.57.6" einops starlette "flatdict==4.0.0" packaging tensorboard \
        "protobuf>=4.25.8,!=5.26.0" "numba==0.63.*" moviepy huggingface_hub transforms3d open3d \
        msgpack msgpack-numpy scikit-learn h5py omegaconf hydra-core opencv-python-headless pillow \
        matplotlib scipy pyyaml tomli imageio
# Isaac Lab, patched for Isaac Sim 6.1 (patches/isaaclab-isaacsim61.patch), and cuRobo.
COPY isaaclab-isaacsim61.patch /opt/isaaclab-isaacsim61.patch
RUN git clone "$ISAACLAB_URL" /opt/IsaacLab && git -C /opt/IsaacLab checkout --detach "$ISAACLAB_COMMIT" \
    && git -C /opt/IsaacLab apply /opt/isaaclab-isaacsim61.patch \
    && ln -sfn /isaac-sim /opt/IsaacLab/_isaac_sim \
    && for p in isaaclab isaaclab_assets isaaclab_contrib isaaclab_mimic isaaclab_rl isaaclab_tasks; do \
        /isaac-sim/python.sh -m pip install -c /opt/isaacsim61_constraints.txt --no-deps \
            -e /opt/IsaacLab/source/$p || exit 1; \
    done
RUN git clone "$CUROBO_URL" /opt/curobo && git -C /opt/curobo checkout --detach "$CUROBO_COMMIT" \
    && cd /opt/curobo && SETUPTOOLS_SCM_PRETEND_VERSION=2.0.0 \
       /isaac-sim/python.sh -m pip install -c /opt/isaacsim61_constraints.txt -e ".[cu12]" --no-build-isolation
# The SONIC encoder/decoder run on onnxruntime (CUDA 12); pyzmq, redis and toml are imported by the
# upstream SONIC action provider.
RUN /isaac-sim/python.sh -m pip install -c /opt/isaacsim61_constraints.txt \
        "onnxruntime-gpu==1.22.0" "pyzmq==27.0.0" "redis==5.2.1" "toml==0.10.2"
RUN /isaac-sim/python.sh -c "\
import onnxruntime as ort, msgpack, numpy, torch, zmq, redis, toml; \
from curobo.motion_planner import MotionPlanner; \
assert 'CUDAExecutionProvider' in ort.get_available_providers(), ort.get_available_providers(); \
print('HUMANOID_IMAGE_OK', ort.__version__, torch.__version__, numpy.__version__)"
ENV PYTHONPATH=/opt/curobo
ENTRYPOINT ["/bin/bash"]
CMD ["-l"]
