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

"""Extracts images from a bagfile."""

import argparse
import concurrent.futures
import os
import sys
from itertools import repeat

import cv2
import cv_bridge
import rosbag
import genpy

import sensor_msgs.msg
import numpy as np

from typing import Tuple

MESSAGE_NUMBER = 0


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

    sys.stdout.write("\rExtracting images: [{0}] {1}%".format(arrow + spaces, int(round(percent * 100))))
    sys.stdout.flush()


def to_cv_image(img_msg: sensor_msgs.msg.Image) -> np.ndarray:
    """
    Convert a ROS image message into a BGR, 16-bit mono or 8-bit mono OpenCV image depending on the input encoding.

    :param img_msg: The ROS image message to be converted.
    :type img_msg: sensor_msgs.msg.Image

    :raises RuntimeError: If the encoding format is unsupported.

    :return: The converted image in OpenCV format.
    :rtype: np.ndarray

    The function converts the given ROS image message to the corresponding OpenCV image format. It supports various
    encoding formats such as 16UC1, mono16, 8UC1, mono8, 8UC3, rgb8, 8UC4, bgra8, and bayer_rggb8. If the image format
    is not supported, a RuntimeError is raised.
    """
    img_data = cv_bridge.CvBridge().imgmsg_to_cv2(img_msg)
    if img_msg.encoding in ['16UC1', 'mono16', '8UC1', 'mono8', '8UC3', 'bgr8']:
        pass  # already in BGR format
    elif img_msg.encoding == 'rgb8':
        img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == '8UC4' or img_msg.encoding == 'bgra8':
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BGRA2BGR)
    elif img_msg.encoding == 'bayer_rggb8':
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_BG2BGR)
    else:
        error_message = (
            f'Unsupported Image format "{img_msg.encoding}"'
            '(Supported are: 16UC1 / mono16, 8UC1 / mono8, 8UC3 / rgb8 / bgr8, 8UC4 / bgra8, bayer_rggb8)'
        )
        print(error_message)  # print the error, because the exception will be silently swallowed by the executor
        raise RuntimeError(error_message)

    return img_data


def process_bag_message(
    bag_message_tuple: Tuple[str, genpy.Message, genpy.Time], args: argparse.Namespace, message_count: int
) -> None:
    """
    Process a single message from a ROS bag file and save the corresponding image file.

    :param bag_message_tuple: A BagMessage(topic, message, timestamp) namedtuple
    :type bag_message_tuple: Tuple[str, genpy.Message, genpy.Time]

    :param args: An argparse namespace object containing the command line arguments.
    :type args: argparse.Namespace

    :param message_count: The total number of messages in the bag file.
    :type message_count: int

    The function writes the image file to the specified image folder based on the
    specified format ('png' or 'jpg') and the message's header stamp.
    """
    # TODO: Theoretically, MESSAGE_NUMBER needs a threading.Lock, but we're not going to bother with this
    global MESSAGE_NUMBER
    progress_bar(MESSAGE_NUMBER, message_count)
    MESSAGE_NUMBER += 1
    (_, msg, _) = bag_message_tuple
    if args.format == 'png':
        filename = os.path.join(args.image_folder, str(msg.header.stamp) + '.png')
        cv2.imwrite(filename, to_cv_image(msg), [cv2.IMWRITE_PNG_COMPRESSION, args.png_compression])
    elif args.format == 'jpg':
        filename = os.path.join(args.image_folder, str(msg.header.stamp) + '.jpg')
        cv2.imwrite(filename, to_cv_image(msg), [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality])
    else:
        error_message = f'Unsupported file format: {args.format}. (Supported are: png, jpg).'
        print(error_message)  # print the error, because the exception will be silently swallowed by the executor
        raise RuntimeError(error_message)


def main():
    """
    Extract images from a bagfile.

    positional arguments:
      bag_name              Path to input bagfile
      image_folder          Path to output folder

    optional arguments:
      -h, --help            show this help message and exit
      -t TOPIC, --topic TOPIC
                            Image topic
      -f FORMAT, --format FORMAT
                            Image format ("png"/"jpg"), default: "png"
      -p PNG_COMPRESSION, --png-compression PNG_COMPRESSION
                            PNG compression (0-9, 0 = off, 1 = best speed (default), 9 = highest compression)
      -j JPG_QUALITY, --jpg-quality JPG_QUALITY
                            JPEG image quality (0-100, default: 95)
      -w WORKERS, --workers WORKERS
                            Number of parallel threads (default: 16)
    """
    parser = argparse.ArgumentParser(description='Extract images from a bagfile.')
    parser.add_argument('bag_name', type=str, help='Path to input bagfile')
    parser.add_argument('image_folder', type=str, help='Path to output folder')
    parser.add_argument('-t', '--topic', type=str, required=True, help='Image topic')
    parser.add_argument('-f', '--format', type=str, default='png', help='Image format ("png"/"jpg"), default: "png"')
    parser.add_argument(
        '-p',
        '--png-compression',
        type=int,
        default=1,
        help='PNG compression (0-9, 0 = off, 1 = best speed (default), 9 = highest compression)',
    )
    parser.add_argument('-j', '--jpg-quality', type=int, default=95, help='JPEG image quality (0-100, default: 95)')
    parser.add_argument('-w', '--workers', type=int, default=16, help='Number of parallel threads (default: 16)')

    args = parser.parse_args()

    # make output folder
    if not os.path.isdir(args.image_folder):
        try:
            os.makedirs(args.image_folder)
        except OSError as e:
            print("ERROR: '{}': {}".format(e.filename, e.strerror))
            sys.exit(1)

    bag = rosbag.Bag(args.bag_name)

    message_count = bag.get_message_count(topic_filters=[args.topic])
    if message_count == 0:
        print('ERROR: no messages on topic {}'.format(args.topic))
        sys.exit(1)

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
        # Note: The executor silently swallows all exceptions!
        executor.map(process_bag_message, bag.read_messages(topics=[args.topic]), repeat(args), repeat(message_count))

    print()  # finish progress bar


if __name__ == '__main__':
    main()
