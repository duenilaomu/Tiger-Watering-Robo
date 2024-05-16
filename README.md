### RoboMaster

1.Switch RoboMaster to direct connection mode(ap)

2.Start up Jetson Nano and connect to RoboMaster with WLAN

3.Run Terminal of Jetson Nano

```shell
$ conda activate robo
```

To switch to virtual environment of the project

4.

```shell
$ sudo chmod 666 /dev/ttyACM0
$ python final.py
```



### YOLOv4-tiny

1.Clone YOLOv4 project to local 

```shell
$ git clone https://github.com/AlexeyAB/darknet.git
```

2. Move `yolov4-tiny_final.weights`  and `yolov4-tiny.cfg`into darknet directory.

3. Specify `Makefile`  and type in terminal

   ```shell
   $ make
   ```

   4. Move `train.data` into `data/`
   5. Specify `cfg/coco`