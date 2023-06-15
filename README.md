# Usage
Note: This is a work in progress and the tool is not generalized. Sensor configuration is hardcoded.



## Installation
1. Clone this repository
2. Install the requirements: ```pip install -r requirements.txt```



## Running the tool
This tool is used to create the dataset for depth testing in `depthai-experiments/depth-test_z-acc_spatial_nose` experiment in batches.
It records the videos from all the cameras that are in the subnet of the device that is running the recording script.

When using more devices - make sure to lower the fps enough so network can handle the load
When all previews are running, press `r` to record the video. Recording automatically stops after framesToRecord number of frames are recorded and the script gives an instruction on
what's the next distance to record.

Example run:
```
python main.py --side center --minDistance 5 --maxDistance 15 --outputDir ~/TestDataset -fps 3  --step 1
```

Usage:
```
usage: main.py [-h] [-fps FPS] [-ftr FRAMESTORECORD] [-side SIDE] [--minDistance MINDISTANCE] [--maxDistance MAXDISTANCE] [--step STEP] [--outputDir OUTPUTDIR]

optional arguments:
  -h, --help            show this help message and exit
  -fps FPS, --fps FPS   FPS to run for all cameras
  -ftr FRAMESTORECORD, --framesToRecord FRAMESTORECORD
                        How many frames should be in the recording
  -side SIDE, --side SIDE
                        Which side is the recording done on
  --minDistance MINDISTANCE
  --maxDistance MAXDISTANCE
  --step STEP
  --outputDir OUTPUTDIR
```