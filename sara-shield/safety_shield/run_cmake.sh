mkdir build 
cd build
sudo cmake -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8/ \
    -DPYTHON_LIBRARY:FILEPATH=/usr/lib/libpython3.8.so \
    ..