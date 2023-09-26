# daa_tools

This package contains helper scripts and tools for the CoPDA project.

## bag_to_imgs

```text
$ rosrun daa_tools bag_to_imgs.py --help
usage: bag_to_imgs.py [-h] -t TOPIC [-f FORMAT] [-p PNG_COMPRESSION] [-j JPG_QUALITY] [-w WORKERS] bag_name image_folder

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
```

The following table shows a comparison of file sizes and compression speeds on 2046x2046 images from the Pylon camera.
All measurements were performed on a Lenovo Thinkpad P15.

| Image format                    | image size | computation speed |
|---------------------------------|------------|-------------------|
| rosbag (lz4 compressed)         | 9 MB       |                   |
| PNG (compression=0, off)        | 12 MB      | 0.2 s/image       |
| PNG (compression=1, best speed) | 4.55 MB    | 0.5 s/image       |
| PNG (compression=9, best size)  | 4.05 MB    | 7.8 s/image       |
| JPEG (quality=95)               | 0.72 MB    | 0.03 s/image      |
| JPEG (quality=80)               | 0.30 MB    | 0.03 s/image      |


## imgs_to_bag

```text
$ rosrun daa_tools imgs_to_bag.py --help
usage: imgs_to_bag.py [-h] -t TOPIC -f FRAME_ID image_folder outbag_name

Convert folder with images to a bagfile.

positional arguments:
  image_folder          Path to image folder (naming: <time stamp in nsec>.png)
  outbag_name           Path to output bagfile

optional arguments:
  -h, --help            show this help message and exit
  -t TOPIC, --topic TOPIC
                        Image topic
  -f FRAME_ID, --frame-id FRAME_ID
                        Frame ID
```
