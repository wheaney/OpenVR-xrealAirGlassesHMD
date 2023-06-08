#!/usr/bin/env bash

# exit when any command fails
set -e

# check out submodules, recusively for nested ones
git submodule update --init --recursive

# build the xrealAir library
XREAL_LIB_BUILD_PATH=modules/xrealAirLinuxDriver/build
if [ ! -d "$XREAL_LIB_BUILD_PATH" ]; then
  mkdir $XREAL_LIB_BUILD_PATH
fi

pushd $XREAL_LIB_BUILD_PATH
cmake ..
make

cd "$(dirs -l -0)" && dirs -c

# build the OpenVR driver
OPENVR_BUILD_PATH=build
if [ ! -d "$OPENVR_BUILD_PATH" ]; then
  mkdir $OPENVR_BUILD_PATH
fi

pushd $OPENVR_BUILD_PATH
cmake ..
make

# move and rename the compiled driver to the driver directory
mv libdriver_air_glasses.so air_glasses/driver_air_glasses.so

# copy the udev rule that's needed for the USB integration
cp -r ../modules/xrealAirLinuxDriver/udev air_glasses/

# bundle up the driver directory
tar -zcvf driver_air_glasses.tar.gz air_glasses

popd