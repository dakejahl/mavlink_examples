# We link against MAVSDK version of mavlink -- which is our QGC_V4.0_ref branch
gcc -std=c99 -I /home/jake/code/wi/MAVSDK/src/third_party/mavlink/include/mavlink/v2.0/common -o example example.c
