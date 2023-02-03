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
mkdir -p $OUTPUT_PATH
mkdir $OUTPUT_PATH/calibration
```

### Retrieve the calibration files from the HoloLens
```
python3 viewer/pv_extrinsic_calibration.py --host 10.10.10.218 --path $OUTPUT_PATH/calibration
python3 viewer/pv_intrinsics_downloader.py --host 10.10.10.218 --path $OUTPUT_PATH/calibration
python3 viewer/rm_calibration_downloader.py --host 10.10.10.218 --path $OUTPUT_PATH/calibration
```

### Record data
```
python3 viewer/record_data.py -o $OUTPUT_PATH -c $OUTPUT_PATH/calibration
```
This will save pointcloud files ("raw_pcd_x.ply") and transformations from the HoloLens to a world coordinate frame ("trafo_x") to the output directory.

### Transform data
```
python3 viewer/transform_to_world_frame.py -f $OUTPUT_PATH
```
This will transform the recorded pointclouds to the world coordinate frame and save them again ("worldpcd_x.ply").

### Visualize data
Pointclouds can be visualized using
```
python3 viewer/visualize_pcls.py -f $OUTPUT_PATH -p world
```
The parameter p gives a pattern to match to extract the correct ply files from the folder.

### Create data set 
```
python3 viewer/slice_data.py -f $OUTPUT_PATH -p world -he 0.0 -t 0.005
```
The parameter -he gives the height at which the data is sliced. The parameter -t controls the tolerance with which data is extracted (he +- t). This will create a csv file dataset in the output folder.

### Visualize data set
```
python3 viewer/visualize_dataset.py -f $OUTPUT_PATH/dataset.csv
```
### Rotate data set
in this example the dataset is rotated around the y axis by 5 degrees
```
python3 viewer/rotate_dataset.py -f $OUTPUT_PATH/dataset.csv -o $OUTPUT_PATH/dataset_rotated.csv -a -5
```

### Additional information
Additional information on the coordinate systems in the HoloLens can be found in the [ResearchApi](https://raw.githubusercontent.com/microsoft/HoloLens2ForCV/main/Docs/ECCV2020-Tutorial/ECCV2020-ResearchMode-Api.pdf).

### Troubleshooting 

-  Broken pipe error 
Hl2ss app should be restarted upon connection error on the client side. It seems like there is no longer data recorded even after restarting the client.

- No connection
Check the configured IP address in the script that should be run. It needs to be changed from the default one. 
