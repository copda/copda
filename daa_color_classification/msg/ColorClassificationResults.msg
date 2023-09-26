Header header

Colors[] colors

sensor_msgs/Image[] histograms

# The cropped object images
sensor_msgs/Image[] object_images

# The bounding boxes of the object images. The coordinates are based on the segmentation image size, not the
# RGB image size.
vision_msgs/BoundingBox2D[] bboxes
