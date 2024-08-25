#!/bin/sh

THIS_SCRIPT_REL_PATH="Tools/gittools/submodule-sync.sh"
if [ -f "$THIS_SCRIPT_REL_PATH" ]; then
    # we're in the ardupilot root directory
    echo "---------------------------------"
    echo "Deleting ardupilot/build"
    echo "Deleting ardupilot/modules"
    echo "---------------------------------"
    rm build -rf
    rm modules -rf
fi

# this copes with moving origin remote to a new git organisation
# we run it 3 times due to the poor handling of recursion
git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive
