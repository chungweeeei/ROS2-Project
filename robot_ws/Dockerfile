FROM ros:kilted

WORKDIR /robot_ws

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    byobu \
    net-tools

COPY . /robot_ws/

RUN <<EOF
echo 'source /opt/ros/kilted/setup.bash' >> /root/.bashrc
echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc
pip3 install -r requirements.txt --break-system-packages
EOF

CMD ["bash"]