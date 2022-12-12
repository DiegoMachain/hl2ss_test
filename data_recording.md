# Instructions for data recording with the HoloLens

## Installation and preparation
Recording is done on a (Linux) host on the same network as the HoloLens

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
Depending on the setup on your machine, some additional packages might be required

### Copy calibration tools to viewer directory
```
cd $HL2SS_PATH/viewer
cp ../tools/* ./
```

## Data recording

### Create output folder and calibration output folder
```
export OUTPUT_PATH=<path-to-desired-output-folder>
mkdir $OUTPUT_PATH
mkdir $OUTPUT_PATH/calibration
```

### Retrieve the calibration files from the HoloLens
```
python3 pv_extrinsic_calibration.py --host 10.10.10.218 --path $OUTPUT_PATH/calibration
python3 pv_intrinsics_downloader.py --host 10.10.10.218 --path $OUTPUT_PATH/calibration
python3 rm_calibration_downloader.py --host 10.10.10.218 --path $OUTPUT_PATH/calibration
```

### Record data
```
python3 record_data.py -o $OUTPUT_PATH -c $OUTPUT_PATH/calibration
```
This will save pointcloud files ("raw_pcd_x.ply") and transformations from the HoloLens to a world coordinate frame ("trafo_x") to the output directory.

### Transform data
```
python3 transform_to_world_frame.py -f $OUTPUT_PATH
```
This will transform the recorded pointclouds to the world coordinate frame and save them again ("worldpcd_x.ply").

### Visualize data
Pointclouds can be visualized using
```
python3 visualize_pcls.py -f $OUTPUT_PATH -p world
```
The parameter p gives a pattern to match to extract the correct ply files from the folder.

### Additional information
Additional information on the coordinate systems in the HoloLens can be found in the [ResearchApi](https://raw.githubusercontent.com/microsoft/HoloLens2ForCV/main/Docs/ECCV2020-Tutorial/ECCV2020-ResearchMode-Api.pdf).

### Troubleshooting 

-  Broken pipe error 
Hl2ss app should be restarted upon connection error on the client side. It seems like there is no longer data recorded even after restarting the client.

- No connection
Check the configured IP address in the script that should be run. It needs to be changed from the default one. 