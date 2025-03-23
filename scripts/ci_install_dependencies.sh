wget -q --no-check-certificate -O ZED_SDK_Linux.run https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Tegra_L4T36.4_v4.2.2.zstd.run && \
chmod +x ZED_SDK_Linux.run && \
    ./ZED_SDK_Linux.run silent skip_od_module skip_python skip_drivers && \
    ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so && \
    rm -rf /usr/local/zed/resources/* ZED_SDK_Linux.run