#!/bin/bash

# install packages
pushd libsigrok
#./autogen.sh
#./configure
#make
make install
popd

pushd libsigrokdecode
#./autogen.sh
#./configure
#make
make install
popd

