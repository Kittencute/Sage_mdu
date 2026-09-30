SHELL := /bin/bash
.SHELLFLAGS := -e -o pipefail -c
ROS_DISTRO ?= jazzy

TARGET_FILE := .sage/current
TARGET ?= $(shell cat $(TARGET_FILE) 2>/dev/null)
SAGE_INSTANCE ?= 1
SERVICE ?=

_TARGET_GOALS := build up shell dev
ifneq (,$(filter $(_TARGET_GOALS),$(MAKECMDGOALS)))
  ifeq ($(strip $(TARGET)),)
    $(error No TARGET selected. Run `make use TARGET=<name>` (see `make list`), or pass TARGET=<name> on the command line.)
  endif
endif

RENDERER := PYTHONPATH=src/fleet_config:src/fleet_unit python3 -m fleet_config
THIRD_PARTY_DIR := src/third_party
PX4_DIR := $(THIRD_PARTY_DIR)/PX4-Autopilot
PX4_SUBMODULES := Tools/simulation/gz src/modules/zenoh/zenoh-pico
PX4_ROS2_DIR := $(THIRD_PARTY_DIR)/px4-ros2-interface-lib
RENDER_DIR := .sage/render/$(TARGET).$(SAGE_INSTANCE)
RUN_DIR := .sage/run/$(SAGE_INSTANCE)
RUN_RECORD := runs/$(shell date -u +%Y%m%dT%H%M%SZ)-$(TARGET).$(SAGE_INSTANCE)
IMAGES := sage-sim sage-phy sage-phy-build sage-dev
COMPOSE_PROJECT := sage$(SAGE_INSTANCE)

DC := docker compose -p $(COMPOSE_PROJECT)
DC_RENDER := docker compose -f $(RENDER_DIR)/compose.yaml --project-directory $(CURDIR)/docker -p $(COMPOSE_PROJECT)
WORKSPACE_DIRS := build install log .ccache

define compose_environment
image=$$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["image"])' $(RENDER_DIR)/fleet.yaml) \
	|| { echo "compose environment failed: operation: read the image from $(RENDER_DIR)/fleet.yaml; cause: reported above" >&2; exit 1; }; \
image_digest=$$(docker image inspect "$$image" --format '{{.Id}}') \
	|| { echo "compose environment failed: operation: inspect image $$image; cause: the image does not exist; run make build TARGET=$(TARGET)" >&2; exit 1; }; \
export SAGE_RENDER_HOST=$(CURDIR)/$(RENDER_DIR); \
export SAGE_RUN_HOST=$(CURDIR)/$(RUN_DIR); \
export SAGE_IMAGE_DIGEST="$$image_digest"; \
export SAGE_VIDEO_GID=$$(getent group video | cut -d: -f3); \
export SAGE_RENDER_GID=$$(getent group render | cut -d: -f3); \
export DOCKER_UID=$$(id -u); \
export DOCKER_GID=$$(id -g)
endef

define build_image
DOCKER_BUILDKIT=1 docker build \
	-f docker/Dockerfile \
	--target $(1) \
	-t $(2):latest \
	$(4) \
	. \
	|| { echo "$(3) failed: operation: build image $(2):latest from docker/Dockerfile target $(1); cause: reported above" >&2; exit 1; }
endef

define render_fleet
rm -rf $(1) && mkdir -p $(1) && $(RENDERER) render --target "$(TARGET)" --instance "$(SAGE_INSTANCE)" --out $(1)
endef

PHYSICAL_ONLY_PKGS := terrascout_hardware livox_ros_driver2 livox_sdk2 gscam2 fpsdk_common fpsdk_apps fpsdk_ros2 fixposition_driver_lib fixposition_driver_ros2 fixposition_driver_msgs rtcm_msgs
SIMULATION_ONLY_PKGS := fleet_simulation
FIRST_PARTY_PATHS := $(shell find src -mindepth 1 -maxdepth 1 -type d -not -name third_party)

ifneq (,$(filter _build-mode,$(MAKECMDGOALS)))
  ifeq (,$(filter simulation physical,$(MODE)))
    $(error build failed: operation: resolve deploy mode for target $(TARGET); cause: rendered fleet.yaml mode is '$(MODE)', expected simulation or physical)
  endif
endif

ifeq ($(MODE),simulation)
  IMAGE := sage-sim
  BUILD_IMAGE := sage-sim
  OTHER_MODE_PKGS := $(PHYSICAL_ONLY_PKGS)
  COLCON_INSTALL := --symlink-install
  FIXPOSITION_SETUP :=
else
  IMAGE := sage-phy
  BUILD_IMAGE := sage-phy-build
  OTHER_MODE_PKGS := $(SIMULATION_ONLY_PKGS)
  COLCON_INSTALL :=
  FIXPOSITION_SETUP := bash $(THIRD_PARTY_DIR)/fixposition_driver/setup_ros_ws.sh &&
endif
COLCON_SKIP := --packages-ignore $(OTHER_MODE_PKGS)

WORKSPACE_MOUNTS := \
	-v $(CURDIR)/src:/workspace/src:rw \
	-v $(CURDIR)/build:/workspace/build:rw \
	-v $(CURDIR)/install:/workspace/install:rw \
	-v $(CURDIR)/log:/workspace/log:rw \
	-v $(CURDIR)/.ccache:/workspace/.ccache:rw

.PHONY: help list use build _build-mode render up down shell dev logs clean

help:
	@echo "SAGE Fleet Management System"
	@echo ""
	@echo "Active target: $(TARGET)"
	@echo ""
	@echo "Commands:"
	@echo "  make list                  # print known targets"
	@echo "  make use TARGET=<name>     # save a target (persists across shells)"
	@echo "  make build                 # render + docker + colcon build + colcon test"
	@echo "  make render                # write what every unit runs to .sage/render/<target>.<instance>/"
	@echo "  make up                    # render the target, record the run in runs/, and start the stack"
	@echo "  make down                  # stop"
	@echo "  make shell                 # shell into a running unit container"
	@echo "  make dev                   # shell on the fleet network in a separate container (teleop, debugging)"
	@echo "  make logs                  # follow compose logs"
	@echo "  make clean                 # remove images and build output"
	@echo ""
	@echo "One-shot target override:"
	@echo "  make build TARGET=terrascout1"
	@echo ""
	@echo "Multi-instance isolation (orthogonal to target):"
	@echo "  SAGE_INSTANCE=2 make up"

list:
	@$(RENDERER) list \
	  | awk -v a='$(TARGET)' '{printf "  %s%s\n", $$1, ($$1==a?"  <- active":"")}'

use:
ifneq (command line,$(origin TARGET))
	$(error Usage: make use TARGET=<name>  (run `make list` to see available targets))
endif
	@mkdir -p $(dir $(TARGET_FILE))
	@echo $(TARGET) > $(TARGET_FILE)
	@echo "Saved target: $(TARGET)"

build:
	@command -v vcs >/dev/null \
		|| { echo "build failed: operation: import third_party.repos into $(THIRD_PARTY_DIR); cause: vcs is not on PATH; install vcstool or vcs2l" >&2; exit 1; }
	@mkdir -p $(THIRD_PARTY_DIR)
	@vcs import --shallow --input third_party.repos $(THIRD_PARTY_DIR) \
		|| { echo "build failed: operation: import third_party.repos into $(THIRD_PARTY_DIR); cause: reported above" >&2; exit 1; }
	@git -C $(PX4_DIR) submodule update --init --recursive $(PX4_SUBMODULES) \
		|| { echo "build failed: operation: initialize PX4 submodules $(PX4_SUBMODULES); cause: reported above" >&2; exit 1; }
	@for patch in $$(find patches -name '*.patch' | sort); do \
		repo=$(THIRD_PARTY_DIR)/$$(dirname $${patch#patches/}); \
		if git -C $$repo apply --reverse --check $(CURDIR)/$$patch 2>/dev/null; then \
			echo "Patch $$patch is already applied to $$repo"; \
		else \
			{ git -C $$repo apply --check $(CURDIR)/$$patch && git -C $$repo apply $(CURDIR)/$$patch; } \
				|| { echo "build failed: operation: apply $$patch to $$repo; cause: reported above" >&2; exit 1; }; \
			echo "Applied $$patch to $$repo"; \
		fi; \
	done
	@touch $(PX4_DIR)/COLCON_IGNORE $(PX4_ROS2_DIR)/examples/COLCON_IGNORE $(PX4_ROS2_DIR)/px4_ros2_py/COLCON_IGNORE $(THIRD_PARTY_DIR)/fixposition_driver/fixposition_driver_ros1/COLCON_IGNORE
	@echo "Resolving deploy mode for target $(TARGET)"
	@tmp=$$(mktemp -d) && trap 'rm -rf "$$tmp"' EXIT && \
		{ $(call render_fleet,"$$tmp") ; } \
			|| { echo "build failed: operation: render target $(TARGET); cause: reported above" >&2; exit 1; } && \
		mode=$$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["mode"])' "$$tmp/fleet.yaml") \
			|| { echo "build failed: operation: read mode from rendered fleet.yaml for target $(TARGET); cause: reported above" >&2; exit 1; } && \
		$(MAKE) --no-print-directory _build-mode TARGET="$(TARGET)" MODE="$$mode"

_build-mode:
	@built=$$(cat install/.sage-mode 2>/dev/null || true); \
	[ -z "$$built" ] || [ "$$built" = "$(MODE)" ] \
		|| { echo "build failed: operation: build the workspace for $(MODE); cause: build/ and install/ hold a $$built build; run make clean" >&2; exit 1; }
	@echo "[1/3] Building $(BUILD_IMAGE):latest from docker/Dockerfile target $(MODE)"
	@$(call build_image,$(MODE),$(BUILD_IMAGE),build)
	@mkdir -p $(WORKSPACE_DIRS)
	@echo "[2/3] Building workspace $(COLCON_INSTALL)"
	@docker run --rm \
		--user $$(id -u):$$(id -g) \
		--workdir /workspace \
		$(WORKSPACE_MOUNTS) \
		-e CCACHE_DIR=/workspace/.ccache \
		-e CC="ccache gcc" -e CXX="ccache g++" \
		--entrypoint bash $(BUILD_IMAGE):latest -c " \
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		$(FIXPOSITION_SETUP) \
		colcon build $(COLCON_INSTALL) --parallel-workers $$(nproc) $(COLCON_SKIP) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
		source install/setup.bash && \
		colcon test --base-paths $(FIRST_PARTY_PATHS) $(COLCON_SKIP) && \
		colcon test-result --verbose \
	" || { echo "build failed: operation: run colcon build and colcon test in $(BUILD_IMAGE):latest; cause: reported above" >&2; exit 1; }
	@echo "$(MODE)" > install/.sage-mode
	@python3 -c 'import glob, json; json.dump([entry for path in sorted(glob.glob("build/*/compile_commands.json")) for entry in json.load(open(path))], open("build/compile_commands.json", "w"))' \
		|| { echo "build failed: operation: merge build/*/compile_commands.json into build/compile_commands.json; cause: reported above" >&2; exit 1; }
ifeq ($(MODE),simulation)
	@echo "[3/3] Building PX4 SITL with zenoh"
	@mkdir -p build/.px4_ccache
	@docker run --rm \
		--user $$(id -u):$$(id -g) \
		--workdir /workspace \
		$(WORKSPACE_MOUNTS) \
		-e CCACHE_DIR=/workspace/build/.px4_ccache \
		-e HOME=/workspace/build \
		--entrypoint bash $(BUILD_IMAGE):latest -c " \
		set -e; \
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		cd /workspace/$(PX4_DIR); \
		mkdir -p build/px4_sitl_zenoh; \
		cd build/px4_sitl_zenoh; \
		if ! cmake ../.. -G Ninja \
			-DCONFIG=px4_sitl_zenoh \
			-DCMAKE_BUILD_TYPE=RelWithDebInfo \
			> /tmp/px4-cmake.log 2>&1; then \
			echo 'PX4 cmake configure failed:'; cat /tmp/px4-cmake.log; exit 1; \
		fi; \
		ninja; \
		test -x bin/px4 && echo 'PX4 binary: '\$$(pwd)/bin/px4 \
	" || { echo "build failed: operation: build PX4 SITL in $(BUILD_IMAGE):latest; cause: reported above" >&2; exit 1; }
endif
ifeq ($(MODE),physical)
	@echo "[3/3] Building $(IMAGE):latest with install/ from docker/Dockerfile target physical-runtime"
	@$(call build_image,physical-runtime,$(IMAGE),build,--build-context install=install)
endif
	@echo "Build complete. Run 'make up' to start."

render:
	@{ $(call render_fleet,$(RENDER_DIR)) ; } \
		|| { echo "render failed: operation: render target $(TARGET) instance $(SAGE_INSTANCE); cause: reported above" >&2; exit 1; }
	@echo "Rendered target $(TARGET) instance $(SAGE_INSTANCE) into $(RENDER_DIR)"

up:
	@running=$$($(DC) ps --status running -q) \
		|| { echo "start failed: operation: inspect Compose project $(COMPOSE_PROJECT); cause: reported above" >&2; exit 1; }; \
	if [ -n "$$running" ]; then \
		echo "start failed: operation: start Compose project $(COMPOSE_PROJECT); cause: the project already has running containers; run make down SAGE_INSTANCE=$(SAGE_INSTANCE)" >&2; \
		exit 1; \
	fi
	@echo "Rendering target $(TARGET) instance $(SAGE_INSTANCE) into $(RENDER_DIR)"
	@{ $(call render_fleet,$(RENDER_DIR)) ; } \
		|| { echo "start failed: operation: render target $(TARGET) instance $(SAGE_INSTANCE); cause: reported above" >&2; exit 1; }
	@mkdir -p $(WORKSPACE_DIRS) $(dir $(RUN_DIR)) $(dir $(RUN_RECORD)) && mkdir $(RUN_RECORD)
	@echo "Recording run in $(RUN_RECORD)"
	@cp -r $(RENDER_DIR) $(RUN_RECORD)/render
	@commit=$$(git rev-parse HEAD) \
		|| { echo "start failed: operation: read the git commit; cause: reported above" >&2; exit 1; }; \
	git status --porcelain > $(RUN_RECORD)/git-status.txt \
		|| { echo "start failed: operation: read the git working tree state; cause: reported above" >&2; exit 1; }; \
	git diff HEAD > $(RUN_RECORD)/git.diff \
		|| { echo "start failed: operation: write the working tree diff against $$commit; cause: reported above" >&2; exit 1; }; \
	dirty=$$([ -s $(RUN_RECORD)/git-status.txt ] && echo true || echo false); \
	printf 'target: %s\ninstance: %s\ncommit: %s\ndirty: %s\n' "$(TARGET)" "$(SAGE_INSTANCE)" "$$commit" "$$dirty" > $(RUN_RECORD)/run.yaml
	@rm -rf $(RUN_DIR) && ln -s $(CURDIR)/$(RUN_RECORD) $(RUN_DIR)
	@gui=$$(python3 -c 'import sys, yaml; gui = yaml.safe_load(open(sys.argv[1]))["gui"]; sys.exit(f"expected a boolean, got {gui!r}") if not isinstance(gui, bool) else print(1 if gui else 0)' $(RENDER_DIR)/fleet.yaml) \
		|| { echo "start failed: operation: read gui from $(RENDER_DIR)/fleet.yaml; cause: reported above" >&2; exit 1; }; \
	if [ "$$gui" = 1 ]; then xhost +local:docker; fi
	@$(compose_environment); \
	$(DC_RENDER) create --force-recreate --remove-orphans \
		|| { echo "start failed: operation: create Compose project $(COMPOSE_PROJECT); cause: reported above" >&2; exit 1; }; \
	containers=$$($(DC_RENDER) ps --all --quiet) \
		|| { echo "start failed: operation: list the containers of Compose project $(COMPOSE_PROJECT); cause: reported above" >&2; exit 1; }; \
	{ echo "images:"; docker inspect --format '  {{index .Config.Labels "com.docker.compose.service"}}: {{.Config.Image}} {{.Image}}' $$containers; } >> $(RUN_RECORD)/run.yaml \
		|| { echo "start failed: operation: record the image of each container in $(RUN_RECORD)/run.yaml; cause: reported above" >&2; exit 1; }; \
	echo "Starting SAGE"; \
	$(DC_RENDER) up --remove-orphans --abort-on-container-exit 2>&1 | tee -i $(RUN_RECORD)/compose.log \
		|| { echo "start failed: operation: run Compose project $(COMPOSE_PROJECT); cause: reported above and in $(RUN_RECORD)/compose.log" >&2; exit 1; }

down:
	@echo "Stopping Compose project $(COMPOSE_PROJECT)"
	@$(DC) down --remove-orphans

shell:
	@test -f $(RENDER_DIR)/fleet.yaml \
		|| { echo "shell failed: operation: read $(RENDER_DIR)/fleet.yaml; cause: the target is not rendered; run make up TARGET=$(TARGET) SAGE_INSTANCE=$(SAGE_INSTANCE)" >&2; exit 1; }
	@service="$(SERVICE)"; \
	if [ -z "$$service" ]; then \
		service=$$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["units"][0]["service"])' $(RENDER_DIR)/fleet.yaml) \
			|| { echo "shell failed: operation: read the first unit service from $(RENDER_DIR)/fleet.yaml; cause: reported above" >&2; exit 1; }; \
	fi; \
	$(compose_environment); \
	$(DC_RENDER) exec "$$service" ros-workspace-exec bash

dev:
	@test -f $(RENDER_DIR)/compose.yaml \
		|| { echo "dev start failed: operation: read $(RENDER_DIR)/compose.yaml; cause: the target is not rendered; run make up TARGET=$(TARGET) SAGE_INSTANCE=$(SAGE_INSTANCE)" >&2; exit 1; }
	@mkdir -p $(WORKSPACE_DIRS)
	@image=$$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["services"]["dev"]["image"])' $(RENDER_DIR)/compose.yaml) \
		|| { echo "dev start failed: operation: read the dev image from $(RENDER_DIR)/compose.yaml; cause: reported above" >&2; exit 1; }; \
	if [ "$$image" = sage-dev:latest ]; then $(call build_image,dev,sage-dev,dev start); fi
	@$(compose_environment); \
	echo "Starting dev container"; \
	$(DC_RENDER) --profile dev run --rm dev

logs:
	@$(DC) logs -f

clean:
	@echo "Removing the containers, the images and workspace output make build creates, and the render and run links make up creates"
	@$(DC) down --volumes --remove-orphans
	@images=$$(docker image ls --format '{{.Repository}}:{{.Tag}}' $(foreach image,$(IMAGES),--filter reference=$(image))); \
	[ -z "$$images" ] || docker image rm $$images \
		|| { echo "clean failed: operation: remove images $(IMAGES); cause: reported above" >&2; exit 1; }
	@rm -rf $(WORKSPACE_DIRS) $(PX4_DIR)/build .sage/render .sage/run
	@echo "Clean complete. Third-party sources in $(THIRD_PARTY_DIR) and run records in runs/ are kept. Run 'make build' to rebuild."
