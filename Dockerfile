FROM espressif/idf:v5.5.1

USER root
RUN <<EOF
    apt-get update
    apt-get install -y \
        clang-format \
        locales \
        sudo \
        udev
    locale-gen en_US.UTF-8
    update-locale LANG=en_US.UTF-8
EOF

RUN <<EOF
    # Add SEGGER J-Link tools (JLinkExe, JLinkGDBServer, JLinkRTTClient, ...)
    wget --post-data 'accept_license_agreement=accepted' \
        -O /tmp/jlink.deb \
        https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb
    apt-get install -y /tmp/jlink.deb
    rm /tmp/jlink.deb
EOF

RUN <<EOF
    # Add user account to sudoers
    echo ubuntu ' ALL = NOPASSWD: ALL' > /etc/sudoers.d/ubuntu
    chmod 0440 /etc/sudoers.d/ubuntu

    # Allow access to USB serial devices (/dev/ttyUSB*, /dev/ttyACM*)
    usermod -aG dialout,plugdev ubuntu
EOF

USER ubuntu

RUN echo "source /opt/esp/entrypoint.sh\nset +e" > ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c"]
SHELL ["/bin/bash", "-c"]
CMD ["bin/bash"]
