# We link against MAVSDK version of mavlink -- which is our QGC_V4.0_ref branch
g++ -std=c++11 -I /home/jake/code/wi/MAVSDK/src/third_party/mavlink/include/mavlink/v2.0/common \
-pthread \
-o example \
example.cpp
