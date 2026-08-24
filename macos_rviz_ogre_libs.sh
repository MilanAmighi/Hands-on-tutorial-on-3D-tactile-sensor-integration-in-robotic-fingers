#!/usr/bin/env bash
# macOS only — workaround for an upstream RoboStack packaging bug.

_ogre_lib_dir="${CONDA_PREFIX}/opt/rviz_ogre_vendor/lib"

if [ -d "${_ogre_lib_dir}" ]; then
    # Externally-linked dependencies of the Ogre libraries. Add a leaf name here
    # if a future rviz2 startup reports another "Library not loaded" dylib.
    for _lib in libfreetype.6.dylib libpng16.16.dylib libz.1.dylib; do
        if [ -e "${CONDA_PREFIX}/lib/${_lib}" ] && [ ! -e "${_ogre_lib_dir}/${_lib}" ]; then
            ln -sf "${CONDA_PREFIX}/lib/${_lib}" "${_ogre_lib_dir}/${_lib}" 2>/dev/null || true
        fi
    done
    unset _lib
fi

unset _ogre_lib_dir
true
