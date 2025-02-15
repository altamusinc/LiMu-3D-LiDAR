
mkdir build

cd build

cmake ..

make

./set_camera_ip <sensor IP address> <new IP address> <image type: 0 - grayscale, 1 - distance, 2 - distance_amplitude>



