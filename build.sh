# We link against MAVSDK version of mavlink -- which is our QGC_V4.0_ref branch
g++ -std=c++11 -I c_library_v2/common \
-pthread \
-o example \
example.cpp
