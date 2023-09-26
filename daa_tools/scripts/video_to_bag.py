#!/usr/bin/env python

# Copyright (c) 2023, DFKI GmbH and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import sys
from os.path import getctime

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge


# usage: video2bag.py [-h] -t TOPIC -f FRAME_ID video_name bag_name


def progress_bar(value: int, endvalue: int, bar_length: int = 20) -> None:
    """
    Progress bar method to track the progress of a task.

    :param value: Current progress value.
    :type value: int

    :param endvalue: Maximum progress value.
    :type endvalue: int

    :param bar_length: Length of the progress bar. Default is 20.
    :type bar_length: int

    :returns: None
    :rtype: None

    Example usage::

        progress_bar(50, 100)
    """
    percent = float(value) / endvalue
    arrow = '-' * int(round(percent * bar_length) - 1) + '>'
    spaces = ' ' * (bar_length - len(arrow))

    sys.stdout.write("\rWriting images to bag: [{0}] {1}%".format(arrow + spaces, int(round(percent * 100))))
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description='Turns video files into rosbags.')
    parser.add_argument('video_name', type=str, help='Path to input video')
    parser.add_argument('bag_name', type=str, help='Path to output bagfile')
    parser.add_argument('-t', '--topic', type=str, required=True, help='bag topic')
    parser.add_argument('-f', '--frame-id', type=str, required=True, help='Frame ID')

    args = parser.parse_args()

    with rosbag.Bag(args.bag_name, 'w', compression='lz4') as bag:
        try:
            unix_date_of_modification = getctime(args.video_name)
        except FileNotFoundError:
            print("Video file not found!")
            return 1

        cap = cv2.VideoCapture(args.video_name)
        if not cap.isOpened():
            print("No video found")
            return 1
        fps = cap.get(cv2.CAP_PROP_FPS)

        cb = CvBridge()

        frame_number = 0
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            progress_bar(frame_number, total_frames)
            frame_number += 1

            timestamps = unix_date_of_modification
            unix_date_of_modification += 1 / fps

            stamp = rospy.rostime.Time.from_sec(timestamps)
            image = cb.cv2_to_imgmsg(frame)

            image.header.stamp = stamp
            image.header.frame_id = args.frame_id

            bag.write(args.topic, image, stamp)

        cap.release()
        print()  # finish progress bar
        return 0


if __name__ == "__main__":
    status = main()
    sys.exit(status)
