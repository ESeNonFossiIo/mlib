#!/bin/bash

DEB_NAME="_uomo_a_terra_1.0.1"
WORKING_DIR="/tmp/${DEB_NAME}/usr/local/bin/"
DEBIAN_DIR="/tmp/${DEB_NAME}/DEBIAN/"


mkdir -p ${WORKING_DIR}
cp -r ./build/* "${WORKING_DIR}"

mkdir -p ${DEBIAN_DIR}
cat _scripts/deb_control >  "${DEBIAN_DIR}/control"


cd "/tmp"
dpkg-deb --build ${DEB_NAME}
