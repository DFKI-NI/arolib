# These definitions happen when arolib config env variables are missing:
AROLIB_AROLIB_ROOT 			?= ${CURDIR}
AROLIB_AROLIB_BUILD_PATH    ?= ${CURDIR}/build
AROLIB_AROLIB_INSTALL_PATH  ?= ${CURDIR}/install

# This happens often when running an arolib command with "sudo" because this doesn't copy over the user's env variables (but "sudo -E" does)
# With these definitions we make some reasonable assumptions
# Otherwise the env script would e.g. set AROLIB_ROOT to "/root/arolib" when things are run without env variables on accident

# docker is often run with sudo; we need to know the real username
REAL_USER := $(if $(SUDO_USER), $(SUDO_USER), $(USER))

# Docker logs in as the current user so that files created inside docker are not owned by root
# mounting /etc/group, passwd and shadow is required for this to work properly
AROLIB_DOCKER_CMD :=	echo ${UBUNTU_VER} && \
						docker run \
						-it \
						-v "${AROLIB_AROLIB_ROOT}":"${AROLIB_AROLIB_ROOT}" \
						-v "${AROLIB_AROLIB_INSTALL_PATH}":"${AROLIB_AROLIB_INSTALL_PATH}" \
						-e AROLIB_AROLIB_ROOT=${AROLIB_AROLIB_ROOT} \
						-e AROLIB_AROLIB_INSTALL_PATH=${AROLIB_AROLIB_INSTALL_PATH} \
						--rm \
						--name "arolib" \
						-u $$(id -u $(REAL_USER)):$$(id -g $(REAL_USER)) \
						-v /etc/group:/etc/group:ro \
						-v /etc/passwd:/etc/passwd:ro \
						-v /etc/shadow:/etc/shadow:ro 

AROLIB_ENV	:=			AROLIB_AROLIB_ROOT="${AROLIB_AROLIB_ROOT}"
						#AROLIB_AROLIB_BUILD_PATH="${AROLIB_AROLIB_BUILD_PATH}"
						#AROLIB_AROLIB_INSTALL_PATH="${AROLIB_AROLIB_INSTALL_PATH}"

AROLIB_SET_ENV := 		${AROLIB_ENV} && . ${AROLIB_AROLIB_ROOT}/scripts/env.sh

AROLIB_BUILD_CMD := 	mkdir -p ${AROLIB_AROLIB_BUILD_PATH}/build; \
						cd ${AROLIB_AROLIB_BUILD_PATH}/build &&\
						cmake ${AROLIB_AROLIB_ROOT} -DCMAKE_INSTALL_PREFIX=${AROLIB_AROLIB_INSTALL_PATH} -DCMAKE_BUILD_TYPE=Release &&\
						make -j$$(nproc --ignore=2) &&\
						make install


.PHONY: build
build: create_dirs
	${AROLIB_SET_ENV} && ${AROLIB_BUILD_CMD}

.PHONY: build/docs
build/docs:
	doxygen Doxyfile

.PHONY: clear
clear:
	rm -r ${AROLIB_AROLIB_BUILD_PATH}/build
	rm -r ${AROLIB_AROLIB_BUILD_PATH}/bin
	rm -r ${AROLIB_AROLIB_BUILD_PATH}/lib

# ${AROLIB_AROLIB_INSTALL_PATH} ${AROLIB_AROLIB_BUILD_PATH}/build:
# 	echo $@
# 	mkdir -p $@
# 	chown -R $$(id -u $(REAL_USER)):$$(id -g $(REAL_USER)) $@

.PHONY: create_dirs
create_dirs: 
	mkdir -p ${AROLIB_AROLIB_INSTALL_PATH}
	chown -R $$(id -u $(REAL_USER)):$$(id -g $(REAL_USER)) ${AROLIB_AROLIB_INSTALL_PATH}
	mkdir -p ${AROLIB_AROLIB_BUILD_PATH}/build
	chown -R $$(id -u $(REAL_USER)):$$(id -g $(REAL_USER)) ${AROLIB_AROLIB_BUILD_PATH}/build
	# ${AROLIB_AROLIB_INSTALL_PATH} ${AROLIB_AROLIB_BUILD_PATH}/build

.PHONY: test/integration
test/integration:
	${AROLIB_SET_ENV} && cd ${AROLIB_AROLIB_BUILD_PATH}/build && ctest --output-on-failure -R integration*

.PHONY: test/units
test/units:
	${AROLIB_SET_ENV} && cd ${AROLIB_AROLIB_BUILD_PATH}/build && ctest --output-on-failure -R unittest*

.PHONY: test
test:	
	${AROLIB_SET_ENV} && cd ${AROLIB_AROLIB_BUILD_PATH}/build && ctest --output-on-failure

.PHONY: docker/build_image/bionic
docker/build_image/bionic: 
	docker build -t arolib/bionic:latest -f Dockerfile.bionic .

.PHONY: docker/build_image/focal
docker/build_image/focal: 
	docker build -t arolib/focal:latest -f Dockerfile.focal .

# if this is run with sudo one has to make sure that $AROLIB_AROLIB_INSTALL_PATH and $AROLIB_AROLIB_ROOT are set as intended
# (because sudo uses a different set of env variables)
# one way is to use the sudo -E, which copies all the env variables of the user
# e.g. `sudo make ...`
.PHONY: docker/build_arolib/bionic
docker/build_arolib/bionic: create_dirs
	${AROLIB_DOCKER_CMD} arolib/bionic:latest bash -c "cd ${AROLIB_AROLIB_ROOT} && make build"

.PHONY: docker/build_arolib/focal
docker/build_arolib/focal: create_dirs
	${AROLIB_DOCKER_CMD} arolib/focal:latest bash -c "cd ${AROLIB_AROLIB_ROOT} && make build"

.PHONY: docker/run/bionic
docker/run/bionic: 
	${AROLIB_DOCKER_CMD} arolib/bionic:latest

.PHONY: docker/run/focal
docker/run/focal: 
	${AROLIB_DOCKER_CMD} arolib/focal:latest
