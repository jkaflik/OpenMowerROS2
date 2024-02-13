# Operation chunk is a part of the robot operation area.
# It's used to divide the operation area into smaller parts for better control and monitoring.
#
# It is defined by a polygon and an area id.

string area_id # id of the zone
geometry_msgs/PolygonStamped area # area of the mowing chunk
nav_msgs/msg/Path paths # path to mow the area
