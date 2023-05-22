#!/usr/bin/env python3

import cv2
import depthai as dai
import contextlib
from  argparse import ArgumentParser
import itertools
from collections import deque 
from pathlib import Path
import time

parser = ArgumentParser()
parser.add_argument("-fps", "--fps", default=3, type=int, required=False,
                    help="FPS to run for all cameras")
parser.add_argument("-ftr", "--framesToRecord", default=10, type=int, required=False, help="How many frames should be in the recording")
parser.add_argument("-side", "--side", type=str, default="center", required=False, help="Which side is the recording done on")
parser.add_argument('--minDistance', type=float, default=0.5)
parser.add_argument('--maxDistance', type=float, default=15)
parser.add_argument('--step', type=float, default=0.25)
parser.add_argument('--outputDir', type=str, default="output")





args = parser.parse_args()

distancesToRunOn = [x for x in range(int(args.minDistance * 100), int(args.maxDistance * 100) + 1, int(args.step * 100))]
distancesToRunOn = [x / 100 for x in distancesToRunOn]
print("Run on distances:", distancesToRunOn)


mapSocketsStr = {
    str(dai.CameraBoardSocket.LEFT): "camb,c",
    str(dai.CameraBoardSocket.RIGHT): "camc,c",
    str(dai.CameraBoardSocket.VERTICAL): "camd,c"
}

fourcc = cv2.VideoWriter_fourcc(*'I420')
sockets = [dai.CameraBoardSocket.LEFT, dai.CameraBoardSocket.RIGHT, dai.CameraBoardSocket.VERTICAL]


class MessageSync:
    def __init__(self, num_queues, min_diff_timestamp, max_num_messages=4, min_queue_depth=3):
        self.num_queues = num_queues
        self.min_diff_timestamp = min_diff_timestamp
        self.max_num_messages = max_num_messages
        # self.queues = [deque() for _ in range(num_queues)]
        self.queues = dict()
        self.queue_depth = min_queue_depth
        # self.earliest_ts = {}

    def add_msg(self, name, msg):
        if name not in self.queues:
            self.queues[name] = deque(maxlen=self.max_num_messages)
        self.queues[name].append(msg)
        # if msg.getTimestampDevice() < self.earliest_ts:
        #     self.earliest_ts = {name: msg.getTimestampDevice()}

        # print('Queues: ', end='')
        # for name in self.queues.keys():
        #     print('\t: ', name, end='')
        #     print(self.queues[name], end=', ')
        #     print()
        # print()

    def get_synced(self):

        # Atleast 3 messages should be buffered
        min_len = min([len(queue) for queue in self.queues.values()])
        if min_len == 0:
            # print('Status:', 'exited due to min len == 0', self.queues)
            return None

        # initializing list of list 
        queue_lengths = []
        for name in self.queues.keys():
            queue_lengths.append(range(0, len(self.queues[name])))
        permutations = list(itertools.product(*queue_lengths))
        # print ("All possible permutations are : " +  str(permutations))

        # Return a best combination after being atleast 3 messages deep for all queues
        min_ts_diff = None
        for indicies in permutations:
            tmp = {}
            i = 0
            for n in self.queues.keys():
                tmp[n] = indicies[i]
                i = i + 1
            indicies = tmp

            acc_diff = 0.0
            min_ts = None
            for name in indicies.keys():
                msg = self.queues[name][indicies[name]]
                if min_ts is None:
                    min_ts = msg.getTimestampDevice().total_seconds()
            for name in indicies.keys():
                msg = self.queues[name][indicies[name]]
                acc_diff = acc_diff + abs(min_ts - msg.getTimestampDevice().total_seconds())

            # Mark minimum
            if min_ts_diff is None or (acc_diff < min_ts_diff['ts'] and abs(acc_diff - min_ts_diff['ts']) > 0.0001):
                min_ts_diff = {'ts': acc_diff, 'indicies': indicies.copy()}
                # print('new minimum:', min_ts_diff, 'min required:', self.min_diff_timestamp)

            if min_ts_diff['ts'] < self.min_diff_timestamp:
                # Check if atleast 5 messages deep
                min_queue_depth = None
                for name in indicies.keys():
                    if min_queue_depth is None or indicies[name] < min_queue_depth:
                        min_queue_depth = indicies[name]
                if min_queue_depth >= self.queue_depth:
                    # Retrieve and pop the others
                    synced = {}
                    for name in indicies.keys():
                        synced[name] = self.queues[name][min_ts_diff['indicies'][name]]
                        # pop out the older messages
                        for i in range(0, min_ts_diff['indicies'][name]+1):
                            self.queues[name].popleft()

                    # print('Returning synced messages with error:', min_ts_diff['ts'], min_ts_diff['indicies'])
                    return synced

        # print('




def createPipeline():
    # Start defining a pipeline
    pipeline = dai.Pipeline()
    # syncNode = pipeline.create(dai.node.Sync)
    streamNames = []
    for socket in sockets:
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(socket)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        cam.setFps(args.fps)
        # Create output
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName(str(socket))
        streamNames.append(xoutRgb.getStreamName())
        cam.isp.link(xoutRgb.input)
        # syncNode.outputs[str(socket)].link(xoutRgb.input)

    return pipeline, streamNames


def get_base_path(mxId, distance):
    return Path(Path(args.outputDir) / f"camera_{mxId}" / f"{distance}m" / args.side / f"1-{mxId}")

with contextlib.ExitStack() as stack:
    deviceInfos = dai.Device.getAllAvailableDevices()
    if len(deviceInfos) == 0:
        raise RuntimeError("No devices found!")

    usbSpeed = dai.UsbSpeed.SUPER
    openVinoVersion = dai.OpenVINO.Version.VERSION_2021_4

    devices = []
    mxIDs = []
    allQueues = {}
    videoWriters = {}
    for deviceInfo in deviceInfos:
        deviceInfo: dai.DeviceInfo
        device: dai.Device = stack.enter_context(dai.Device(openVinoVersion, deviceInfo, usbSpeed))
        devices.append(device)
        print("===Connected to ", deviceInfo.getMxId())
        mxId = deviceInfo.getMxId()
        mxIDs.append(mxId)
        cameras = device.getConnectedCameras()
        usbSpeed = device.getUsbSpeed()
        eepromData = device.readCalibration2().getEepromData()
        print("   >>> MXID:", mxId)
        print("   >>> Num of cameras:", len(cameras))
        pipeline, streamNames = createPipeline()
        device.startPipeline(pipeline)

        outputQueues = {}
        videoWriters[mxId] = {}
        basePath = get_base_path(mxId, distancesToRunOn[0])
        basePath.mkdir(parents=True, exist_ok=True)
        for name in streamNames:
            queue = device.getOutputQueue(name, 1, blocking=False)
            outputQueues[name] = queue
            videoPath = basePath / f"{mapSocketsStr[name]}.avi"
            videoWriters[mxId][name] = cv2.VideoWriter(str(videoPath.absolute()), fourcc, args.fps, (1920, 1200))
        allQueues[mxId] = outputQueues
    frameCount = 0
    distanceId = 0
    recording = False
    print(f"First measurement needs to be on {distancesToRunOn[distanceId]}m")
    syncMessages = MessageSync(3, 0.003, 20, 2)
    while True:
        for mxId in mxIDs:
            frames = {}
            framesList = []
            for stream in allQueues[mxId].keys():
                time.sleep(0.001)
                frame = allQueues[mxId][stream].tryGet()
                if frame is not None:
                    syncMessages.add_msg(stream, frame)
                frames = syncMessages.get_synced()
                if frames is None:
                    continue
                if recording:
                    frameCount += 1
                for key in frames.keys():
                    frames[key] = frames[key].getCvFrame()
                    framesList.append(frames[key])
                mergedImage = cv2.hconcat(framesList)
                mergedImage = cv2.resize(mergedImage, (900, 150))
                cv2.imshow(mxId, mergedImage)
                if not recording:
                    continue
                if frameCount < 10:
                    videoWriters[mxId][stream].write(frames[stream])

        if frameCount >= 10:
            recording = False
            frameCount = 0
            print(f"Recording on {distancesToRunOn[distanceId]}m is done.")
            distanceId += 1
            if distanceId >= len(distancesToRunOn):
                print("All measurements are done.")
                break
            print(f"Now measure on distance {distancesToRunOn[distanceId]}m.")
            for mxId in mxIDs:
                for stream in allQueues[mxId].keys():
                    basePath = get_base_path(mxId, distancesToRunOn[distanceId])
                    basePath.mkdir(parents=True, exist_ok=True)
                    videoPath = basePath / f"{mapSocketsStr[stream]}.avi"
                    videoWriters[mxId][stream].release()
                    videoWriters[mxId][stream] = cv2.VideoWriter(str(videoPath.absolute()), fourcc, args.fps, (1920, 1200))
        cvKey = cv2.waitKey(1)
        if cvKey == ord('q'):
            break
        if cvKey == ord('r'):
            print("Start recording")
            recording = True
            frameCount = 0