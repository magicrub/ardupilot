#!/bin/bash
# build all targets for PX4
# This helps when doing large merges
# Andrew Tridgell, February 2013

. config.mk

set -e
set -x

git submodule init
git submodule update

for d in ArduPlane; do
    pushd $d
    make px4-clean
    popd
done

echo "Testing ArduPlane build"
pushd ArduPlane
make px4
popd

exit 0
