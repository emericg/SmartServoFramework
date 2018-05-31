prefix=${CMAKE_INSTALL_PREFIX}
exec_prefix=${EXEC_INSTALL_PREFIX}
libdir=${LIB_INSTALL_DIR}
includedir=${INCLUDE_INSTALL_DIR}

Name: ${PROJECT_NAME}
Description: SmartServoFramework is a multi-platform C++ framework used to drive "smart servo" devices
URL: https://github.com/emericg/SmartServoFramework
Version: ${PROJECT_VERSION}
Libs: -L${LIB_INSTALL_DIR} -lsmartservoframework
Cflags: -I${INCLUDE_INSTALL_DIR}
