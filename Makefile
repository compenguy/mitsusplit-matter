V=@
PWD=$(shell pwd)

# Docker Variables
APP_NAME=mitsusplit-matter
DOCKER_TAG=${APP_NAME}:latest
DOCKER_USERNAME=builder
USER_ID=$(shell id -u)
GROUP_ID=$(shell id -g)
docker_run=docker run \
		-it \
		--rm \
		--mount type=bind,source="$(PWD)",target=/home/$(DOCKER_USERNAME) \
		--env USER_NAME=$(DOCKER_USERNAME) \
		--env USER_ID=$(USER_ID) \
		--env GROUP_ID=$(GROUP_ID) \
		--env HOME="/home/$(DOCKER_USERNAME)" \
		--workdir="/home/$(DOCKER_USERNAME)" \
		"$(DOCKER_TAG)" \
		/opt/entrypoint.sh \
		"$(1)"

# Targets
BINS=build/chip-tool
STAMPS=.docker-build.stamp

# Application and test configuration
CHIP_TEST_SETUP_PIN_CODE=20202021
CHIP_TEST_DISCRIMINATOR=3840

NODE_ID=0xC0014112
WIFI_SSID=
WIFI_PSK=
COMMISSION_PASSCODE=$(CHIP_TEST_SETUP_PIN_CODE)
COMMISSION_DISCRIMINATOR=$(CHIP_TEST_DISCRIMINATOR)

# Start build rules
.PHONY: all
all: $(BINS)

.PHONY: commission
commision: build/chip-tool
	@$(if $(NODE_ID),$(shell echo node-id: $(NODE_ID)),$(error Environment variable NODE_ID required for commissioning))
	@$(if $(WIFI_SSID),$(shell echo ssid: $(WIFI_SSID)),$(error Environment variable WIFI_SSID required for commissioning))
	@$(if $(WIFI_PSK),$(shell echo psk: $(WIFI_PSK)),$(error Environment variable WIFI_PSK required for commissioning))
	@$(if $(COMMISSION_PASSCODE),$(shell echo passcode: $(COMMISSION_PASSCODE)),$(error Environment variable COMMISION_PASSCODE required for commissioning))
	@$(if $(COMMISSION_DISCRIMINATOR),$(shell echo discriminator: $(COMMISSION_DISCRIMINATOR)),$(error Environment variable COMMISION_DISCRIMINATOR required for commissioning))
	$(V)build/chip-tool pairing ble-wifi $(NODE_ID) $(WIFI_SSID) $(WIFI_PSK) $(COMMISION_PASSCODE) $(COMMISSION_DISCRIMINATOR)

.PHONY: docker-build
docker-build: .docker-build.stamp
.docker-build.stamp: docker/Dockerfile docker/entrypoint.sh
	# Building docker container
	$(V)docker build -t "$(DOCKER_TAG)" $(<D)
	@touch $@

.PHONY: bootstrap
bootstrap: .cipd-cache-dir/tagcache.db
.cipd-cache-dir/tagcache.db: connectedhomeip/connectedhomeip/scripts/bootstrap.sh docker-build
	# Bootstrapping Matter
	$(V)$(call docker_run,source $<)

build/chip-tool: connectedhomeip/connectedhomeip/examples/chip-tool/BUILD.gn connectedhomeip/connectedhomeip/examples/chip-tool/main.cpp  connectedhomeip/connectedhomeip/scripts/examples/gn_build_example.sh bootstrap docker-build
	# Building chip-tool
	$(V)$(call docker_run,connectedhomeip/connectedhomeip/scripts/examples/gn_build_example.sh connectedhomeip/connectedhomeip/examples/chip-tool $(@D))

.PHONY: shell
shell: docker/Dockerfile docker/entrypoint.sh
	$(V)$(call docker_run,bash)

.PHONY: clean
clean:
	$(V)rm -f $(STAMPS)
	$(V)rm -f $(BINS)
	$(V)rm -fR .cipd-cache-dir
