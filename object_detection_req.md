# Object Detection Request

Here we describe the object information required for downstreaming task like task planning and grasping. These are basic requirements. If you want to add more challenges in object detection module, feel free to modify this.


## For task planning
### object color
- ros message type: `std_msgs/ColorRGBA`,

```
## For example, to publish a red color message:
from std_msgs.msg import ColorRGBA

color_msg = ColorRGBA()
color_msg.r = 1.0  # Red
color_msg.g = 0.0  # Green
color_msg.b = 0.0  # Blue
color_msg.a = 1.0  # Alpha (opacity)
```

### object shape
- ros message type: `std_msgs/String`, "square", "triangle", "semicircle"



## For grasping
### object center position: x,y,z
- ros message type: `geometry_msgs/Point`,

```
from geometry_msgs.msg import Point

center = Point()
center.x = 1.0  # X-coordinate
center.y = 2.0  # Y-coordinate
center.z = 3.0  # Z-coordinate
```


### container position: x,y,z (To be discussed, it is optional)
- ros message type: `geometry_msgs/Point`





