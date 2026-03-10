pull image


set udev rules on host


use xhost+


proper docker command:

```bash
docker run \
    --name ros2_doc \
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --volume /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume /etc/passwd:/etc/passwd:ro \
    --volume /etc/group:/etc/group:ro \
    algrish2/ros2-kilted:ver-1
```


install astral 

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

clone repo into src:

```bash
cd /home/projects/ros2_ws/src
git clone https://github.com/grishnakov/ros2_tracking.git
cd ..
```

install proper venv for this project/package.
```bash
cd /home/projects/ros2_ws
uv venv .venv --seed
source .venv/bin/activate
uv pip install \
  colcon-common-extensions \
  numpy \
  opencv-python \
  PyYAML \
  depthai \
  depthai-nodes \
  lap \
  ultralytics \
  "git+https://github.com/LiamBindle/PyVESC" \
  pyserial
```


update the pyvesc package by commenting out lines 41 and 42 in `.venv/lib/python3.12/site-packages/pyvesc/VESC/VESC.py`


```python

        # check firmware version and set GetValue fields to old values if pre version 3.xx
        version = self.get_firmware_version()
        # if int(version.split('.')[0]) < 3:       <- This line
        #    GetValues.fields = pre_v3_33_fields   <- This line
```
