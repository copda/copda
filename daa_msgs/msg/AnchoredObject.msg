Header header

# A globally unique instance id. This integer has to be unique across all
# objects, not just the objects of a given class.
int64 instance_id

# The human-readable object name.
string name

vision_msgs/ObjectHypothesisWithPose classification
vision_msgs/BoundingBox3D bbox
