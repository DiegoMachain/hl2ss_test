# Instructions

### Set environment path
```
export HL2SS_PATH=<path-to-hl2ss-main-folder>
```

### Python installations
Install python packages 
```
pip3 install pycryptodome gnupg opencv-python pynput open3d
```
Fix the av bug (https://github.com/PyAV-Org/PyAV/issues/978) and install:
```
sudo apt-get install -y python-dev pkg-config
sudo apt-get install -y \
    libavformat-dev libavcodec-dev libavdevice-dev \
    libavutil-dev libswscale-dev libswresample-dev libavfilter-dev
pip3 install av --no-binary av
```

### Copy calibration tools to viewer directory
```
cd $HL2SS_PATH/viewer
cp ../tools/* ./
```

### Create output and calibration output folder
```
mkdir output
mkdir ./output/calibration
mkdir ./output/run1
mkdir ./output/run2
```

### Retrieve the calibration files from the HoloLens
```
python3 pv_extrinsic_calibration.py --host 10.10.10.218 --path ./output/calibration
python3 pv_intrinsics_downloader.py --host 10.10.10.218 --path ./output/calibration
python3 rm_calibration_downloader.py --host 10.10.10.218 --path ./output/calibration
```

### Record data
First change the output directory to run1 or run2 etc in line 81 of open3d_integrator_pv_rosbag.py. Start the hl2ss application on the HoloLens. I always simply deployed from the workstation directly and kept it on the USB cable. The sln file should be located in "~/Documents/toni/hl2ss" (or similar). Then record some point clouds in different poses by moving the HoloLens around. Preferably pointing at the ceiling or walls such that there are many readings. Don't move too fast at the moment, since the recording and saving is very slow at the moment bc. of the number of points (I will look later at that). The terminal of the client will show whenever a new pcl has been saved, ~15-20 point clouds for one run would be great.
The client is connected via 
```
python3 open3d_integrator_pv_rosbag.py
```

### Troubleshooting 

-  Broken pipe error 
Hl2ss app should be restarted upon connection error on the client side. It seems like there is no longer data recorded even after restarting the client.

- No connection
Check the configured IP address in the script that should be run. It needs to be changed from the default one. 