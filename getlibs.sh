#! /bin/bash

#! /bin/bash

# Project are included in the repo (TODO: separate)
# # Clone libsigrok
# git clone https://github.com/sigrokproject/libsigrok.git

# # Clone libsigrokdecode
# git clone https://github.com/sigrokproject/libsigrokdecode.git

# # Clone pulseview
# git clone https://github.com/sigrokproject/pulseview.git


# install libsigrok
pushd libsigrok
./autogen.sh
./configure
make
sudo make install
popd

# install libsigrokdecode
pushd libsigrokdecode
./autogen.sh
./configure
make
sudo make install
popd

# install pulseview
pushd pulseview
cmake .
make
popd

