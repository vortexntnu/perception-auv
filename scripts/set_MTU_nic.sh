sudo ip link set enP5p1s0f3 down
sudo ip link set enP5p1s0f3 mtu 9000
sudo ip link set enP5p1s0f3 up
sudo systemctl restart NetworkManager
ip link show enP5p1s0f3
