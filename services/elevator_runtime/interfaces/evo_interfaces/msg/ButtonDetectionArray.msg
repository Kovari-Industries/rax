# All buttons found in a single camera frame.

# Camera frame and original EVJ1 capture time (Unix clock), not inference completion.
std_msgs/Header header

# Original EVJ1 monotonic capture time and publication generation. Join perception
# only when source, capture time, and generation all agree; do not join latest results.
uint64 capture_monotonic_ns
uint64 source_generation

# Which camera produced the frame: "base", "left_arm" or "right_arm".
string source

# Dimensions of the image the boxes are expressed in, so consumers can
# normalise without having to subscribe to the image topic itself.
uint32 image_width
uint32 image_height

# Wall-clock inference cost for this frame, for latency budgeting.
float32 inference_ms

# Exact encoded camera capture used for recognition, not the annotated output.
# Bounded to 8 MiB. Geometry consumers pass this snapshot to depth; they must not
# join these boxes to a later live camera frame.
uint8[] source_jpeg

ButtonDetection[] detections
