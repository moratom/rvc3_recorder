#!/usr/bin/env python3

import cv2
import depthai as dai
import contextlib
from  argparse import ArgumentParser
from pathlib import Path

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

fourcc = cv2.VideoWriter_fourcc(*'XVID')
sockets = [dai.CameraBoardSocket.LEFT, dai.CameraBoardSocket.RIGHT, dai.CameraBoardSocket.VERTICAL]

def createPipeline():
    # Start defining a pipeline
    pipeline = dai.Pipeline()
    syncNode = pipeline.create(dai.node.Sync)
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
        cam.isp.link(syncNode.inputs[str(socket)])
        syncNode.outputs[str(socket)].link(xoutRgb.input)

    return pipeline, streamNames


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
        for name in streamNames:
            queue = device.getOutputQueue(name, 4, blocking=True)
            outputQueues[name] = queue
            videoPath = Path(Path(args.outputDir) / f"camera_{mxId}" / args.side / "3.0m" / f"1-{mxId}/{mapSocketsStr[name]}.avi")
            videoPath.parent.mkdir(parents=True, exist_ok=True)
            videoWriters[mxId][name] = cv2.VideoWriter(str(videoPath.absolute()), fourcc, args.fps, (1920, 1200))
        allQueues[mxId] = outputQueues
    i = 0
    while True:
        i += 1
        for mxId in mxIDs:
            frames = {}
            framesList = []
            for stream in allQueues[mxId].keys():
                frames[stream] = allQueues[mxId][stream].get().getCvFrame()
                framesList.append(frames[stream])
                if i < 10:
                    videoWriters[mxId][stream].write(frames[stream])
                else:
                    print("No more recording")
            mergedImage = cv2.hconcat(framesList)
            mergedImage = cv2.resize(mergedImage, (900, 150))
            cv2.imshow(mxId, mergedImage)

        if cv2.waitKey(1) == ord('q'):
            break