.EXPORT_ALL_VARIABLES:

# These definitions happen when arolib config env variables are missing:
AROLIB_AROLIB_ROOT 			?= ${CURDIR}
AROLIB_AROLIB_BUILD_PATH    ?= ${CURDIR}/build
AROLIB_AROLIB_INSTALL_PATH  ?= ${CURDIR}/install

RELEASE_PATH := ${AROLIB_AROLIB_BUILD_PATH}/release
BUILD_PATH := ${RELEASE_PATH}/build

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

AROLIB_BUILD_CMD := 	mkdir -p ${BUILD_PATH}; \
						cd ${BUILD_PATH} &&\
						cmake ${AROLIB_AROLIB_ROOT} -DCMAKE_INSTALL_PREFIX=${AROLIB_AROLIB_INSTALL_PATH} -DCMAKE_BUILD_TYPE=Release &&\
						make -j$$(nproc --ignore=2) &&\
						make install


.PHONY: build
build: create_dirs
	${AROLIB_BUILD_CMD}

# Linux install locations: https://stackoverflow.com/questions/41360283/subfolders-in-usr-local-lib
.PHONY: install
install:
	rm -rf /usr/local/include/arolib && cp -R ${AROLIB_AROLIB_INSTALL_PATH}/include/arolib /usr/local/include
	cp ${AROLIB_AROLIB_INSTALL_PATH}/lib/*.so /usr/local/lib
	echo "/usr/local/lib" > /etc/ld.so.conf.d/arolib.conf && ldconfig

.PHONY: uninstall
uninstall:
	rm -rf /usr/local/include/arolib
	cd /usr/local/lib && rm -f libAROLIB_* 

.PHONY: build/docs
build/docs:
	doxygen Doxyfile

.PHONY: clear
clear:
	rm -r ${RELEASE_PATH}/build
	rm -r ${RELEASE_PATH}/bin
	rm -r ${RELEASE_PATH}/lib

.PHONY: create_dirs
create_dirs: 
	mkdir -p ${AROLIB_AROLIB_INSTALL_PATH}
	chown -R $$(id -u $(REAL_USER)):$$(id -g $(REAL_USER)) ${AROLIB_AROLIB_INSTALL_PATH}
	mkdir -p ${BUILD_PATH}
	chown -R $$(id -u $(REAL_USER)):$$(id -g $(REAL_USER)) ${BUILD_PATH}

.PHONY: test/integration
test/integration:
	cd ${BUILD_PATH} && ctest --output-on-failure -R integration*

.PHONY: test/units
test/units:
	cd ${BUILD_PATH} && ctest --output-on-failure -R unittest*

.PHONY: test
test:	
	cd ${BUILD_PATH} && ctest --output-on-failure
	
.PHONY: docker/build_image/focal
docker/build_image/focal: 
	docker build -t arolib/focal:latest -f Dockerfile.focal .

# if this is run with sudo one has to make sure that $AROLIB_AROLIB_INSTALL_PATH and $AROLIB_AROLIB_ROOT are set as intended
# (because sudo uses a different set of env variables)
# one way is to use the sudo -E, which copies all the env variables of the user
# e.g. `sudo make ...`


.PHONY: docker/build_arolib/focal
docker/build_arolib/focal: create_dirs
	${AROLIB_DOCKER_CMD} arolib/focal:latest bash -c "cd ${AROLIB_AROLIB_ROOT} && make build"


.PHONY: docker/run/focal
docker/run/focal: 
	${AROLIB_DOCKER_CMD} arolib/focal:latest
