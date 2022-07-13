V = @
PWD = $(shell pwd)
APP_NAME = mitsusplit-matter

# Targets
BINS = build/chip-tool build/$(APP_NAME).bin
STAMPS = build/.docker-build.stamp build/.firmware-flash.stamp

# Local/site-specific settings
-include local.mak
ESP_TARGET ?= esp32c3
ESP_MATTER_DEVICE ?= esp32c3_nodemcu
ESP_TTY ?= /dev/ttyUSB1
WIFI_SSID ?=
WIFI_PSK ?=

# Hardware type and SDK
ESP_ENV_SCRIPT = esp-idf/export.sh
MATTER_ENV_SCRIPT = connectedhomeip/connectedhomeip/scripts/activate.sh

# Application and test configuration
CHIP_TEST_SETUP_PIN_CODE ?= 20202021
CHIP_TEST_DISCRIMINATOR ?= 3840

# Unique ID assigned to this particular device in the network
NODE_ID ?= 0x7283
COMMISSION_PASSCODE = $(CHIP_TEST_SETUP_PIN_CODE)
COMMISSION_DISCRIMINATOR = $(CHIP_TEST_DISCRIMINATOR)

# Docker Variables
DOCKER_TAG = ${APP_NAME}:latest
DOCKER_USERNAME = builder
USER_ID = $(shell id -u)
GROUP_ID = $(shell id -g)
TTY_GROUP_NAME = $(shell stat --format=%G $(ESP_TTY))
TTY_GROUP_ID = $(shell getent group $(TTY_GROUP_NAME) | cut -d: -f3)
docker_run = docker run \
		-it \
		--rm \
		--mount type=bind,source="$(PWD)",target=/home/$(DOCKER_USERNAME) \
		--env USER_NAME=$(DOCKER_USERNAME) \
		--env USER_ID=$(USER_ID) \
		--env GROUP_ID=$(GROUP_ID) \
		--env TTY_GROUP_NAME=$(TTY_GROUP_NAME) \
		--env TTY_GROUP_ID=$(TTY_GROUP_ID) \
		--env HOME="/home/$(DOCKER_USERNAME)" \
		--env IDF_TARGET="$(ESP_TARGET)" \
		--env ESP_MATTER_PATH="/home/$(DOCKER_USERNAME)" \
		$(if $(ESP_MATTER_DEVICE),--env ESP_MATTER_DEVICE_PATH="/home/$(DOCKER_USERNAME)/device_hal/device/$(ESP_MATTER_DEVICE)",) \
		$(if $(ESP_TTY),--device=$(ESP_TTY),) \
		--workdir="/home/$(DOCKER_USERNAME)" \
		"$(DOCKER_TAG)" \
		/opt/entrypoint.sh \
		"$(1)"

# Start build rules
.PHONY: all
all: $(BINS)

.PHONY: commission
commission: build/chip-tool
	@echo $(if $(NODE_ID),node-id: $(NODE_ID),$(error Environment variable NODE_ID required for commissioning))
	@echo $(if $(WIFI_SSID),ssid: $(WIFI_SSID),$(error Environment variable WIFI_SSID required for commissioning))
	@echo $(if $(WIFI_PSK),psk: $(WIFI_PSK),$(error Environment variable WIFI_PSK required for commissioning))
	@echo $(if $(COMMISSION_PASSCODE),passcode: $(COMMISSION_PASSCODE),$(error Environment variable COMMISION_PASSCODE required for commissioning))
	@echo $(if $(COMMISSION_DISCRIMINATOR),discriminator: $(COMMISSION_DISCRIMINATOR),$(error Environment variable COMMISION_DISCRIMINATOR required for commissioning))
	$(V)build/chip-tool pairing ble-wifi $(NODE_ID) $(WIFI_SSID) $(WIFI_PSK) $(COMMISSION_PASSCODE) $(COMMISSION_DISCRIMINATOR)

.PHONY: docker-build
docker-build: build/.docker-build.stamp
build/.docker-build.stamp: docker/Dockerfile docker/entrypoint.sh
	# Building docker container
	$(V)docker build -t "$(DOCKER_TAG)" $(<D)
	@mkdir -p $(@D)
	@touch $@

.PHONY: bootstrap
bootstrap: .cipd-cache-dir/tagcache.db .espressif/idf-env.json
.cipd-cache-dir/tagcache.db: connectedhomeip/connectedhomeip/scripts/bootstrap.sh | docker-build
	# Bootstrapping Matter
	$(V)$(call docker_run,source $<)
	@touch $@

.espressif/idf-env.json: esp-idf/install.sh
	# Bootstrapping ESP-IDF
	$(V)$(call docker_run,cd $(<D) && source $(<F) $(ESP_TARGET))
	@touch $@


build/chip-tool: connectedhomeip/connectedhomeip/examples/chip-tool/BUILD.gn connectedhomeip/connectedhomeip/examples/chip-tool/main.cpp  connectedhomeip/connectedhomeip/scripts/examples/gn_build_example.sh | bootstrap docker-build
	# Building chip-tool
	@mkdir -p $(@D)
	$(V)$(call docker_run,connectedhomeip/connectedhomeip/scripts/examples/gn_build_example.sh connectedhomeip/connectedhomeip/examples/chip-tool $(@D))

.PHONY: build/$(APP_NAME).bin
build/$(APP_NAME).bin: $(ESP_ENV_SCRIPT) $(MATTER_ENV_SCRIPT) | bootstrap docker-build
	# Building firmware
	@mkdir -p $(@D)
	$(V)$(call docker_run,source $(ESP_ENV_SCRIPT) && source $(MATTER_ENV_SCRIPT) && idf.py --build-dir \$${ESP_MATTER_PATH}/build --project-dir \$${ESP_MATTER_PATH}/src build)

.PHONY: flash
flash: build/.firmware-flash.stamp
build/.firmware-flash.stamp: build/$(APP_NAME).bin | bootstrap docker-build
	$(V)$(call docker_run,source $(ESP_ENV_SCRIPT) && idf.py --build-dir \$${ESP_MATTER_PATH}/build --project-dir \$${ESP_MATTER_PATH}/src flash -p $(ESP_TTY))

.PHONY: monitor
monitor: build/.firmware-flash.stamp | bootstrap docker-build
	$(V)$(call docker_run,source $(ESP_ENV_SCRIPT) && idf.py --build-dir \$${ESP_MATTER_PATH}/build --project-dir \$${ESP_MATTER_PATH}/src flash -p $(ESP_TTY))

.PHONY: shell
shell: docker/Dockerfile docker/entrypoint.sh | bootstrap docker-build
	$(V)$(call docker_run,bash)

.PHONY: clean
clean:
	$(V)rm -f $(STAMPS)
	$(V)rm -f $(BINS)
	$(V)rm -fR .cipd-cache-dir
