# We link against MAVSDK version of mavlink -- which is our QGC_V4.0_ref branch
# g++ -std=c++11 -I c_library_v2/common \
# -pthread \
# -o camera \
# examples/camera_device.cpp

g++ -std=c++11 -I c_library_v2/common -I c_library_v2/a2z \
-pthread \
-o winch_test \
examples/a2zwinch.cpp