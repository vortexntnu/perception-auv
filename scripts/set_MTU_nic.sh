sudo ip link set enP5p1s0f0 down
sudo ip link set enP5p1s0f0 mtu 9000
sudo ip link set enP5p1s0f0 up
sudo nmcli connection modify "Wired connection 3" 802-3-ethernet.mtu 9000
sudo systemctl restart NetworkManager
ip link show enP5p1s0f0
