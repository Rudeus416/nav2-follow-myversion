#!/bin/bash

source /etc/profile
MV_CAM_RUNENV_PATH=$MVCAM_COMMON_RUNENV
cd  ${MV_CAM_RUNENV_PATH}

echo "create link to dynamic library"
if [ -d "${MV_CAM_RUNENV_PATH}/64" ]; then
ldconfig -n ./64
fi

if [ -d "${MV_CAM_RUNENV_PATH}/32" ]; then
ldconfig -n ./32
fi

if [ -d "${MV_CAM_RUNENV_PATH}/armhf" ]; then
ldconfig -n ./armhf
fi

if [ -d "${MV_CAM_RUNENV_PATH}/aarch64" ]; then
ldconfig -n ./aarch64
fi

if [ -d "${MV_CAM_RUNENV_PATH}/arm-none" ]; then
ldconfig -n ./arm-none
fi
