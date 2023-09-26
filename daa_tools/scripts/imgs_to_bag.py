#!/usr/bin/env python3

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

"""Converts a folder of png images into a bagfile."""

import argparse
import re
from glob import glob

import cv2
import cv_bridge
import rosbag
import rospy
from tqdm import tqdm


def main():
    parser = argparse.ArgumentParser(description='Convert folder with images to a bagfile.')
    parser.add_argument('image_folder', type=str, help='Path to image folder (naming: <time stamp in nsec>.png)')
    parser.add_argument('outbag_name', type=str, help='Path to output bagfile')
    parser.add_argument('-t', '--topic', type=str, required=True, help='Image topic')
    parser.add_argument('-f', '--frame-id', type=str, required=True, help='Frame ID')

    args = parser.parse_args()

    # file name pattern: 1598972685194626227.png
    filename_pattern = '/[0-9]*.png'
    timestamp_re = re.compile(r'([0-9]+)\.png')
    cvb = cv_bridge.CvBridge()

    with rosbag.Bag(args.outbag_name, 'w', compression='lz4') as outbag:
        filenames = sorted(glob(args.image_folder + filename_pattern))
        for filename in tqdm(filenames):
            # read image file to message with cv_bridge
            img = cv2.imread(filename, cv2.IMREAD_COLOR)
            if img is None:
                print('could not read image: ' + filename)
                continue
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            encoding = 'rgb8'

            # fill message header
            match = timestamp_re.search(filename)
            if match is not None:
                total_nsecs = int(match.group(1))
                msg = cvb.cv2_to_imgmsg(img, encoding=encoding)
                msg.header.stamp = rospy.Time(nsecs=total_nsecs)
                msg.header.frame_id = args.frame_id

                outbag.write(args.topic, msg, msg.header.stamp)


if __name__ == '__main__':
    main()
