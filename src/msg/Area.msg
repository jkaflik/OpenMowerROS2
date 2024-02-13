# Definition of the area
# The area is defined by a polygon
# The area can be of 3 types:
# - Exclusion: the area is forbidden
# - Navigation: the area is a navigation area
# - Operation: the area is an operation area (e.g. lawn mowing area)

uint8 TYPE_EXCLUSION = 0
uint8 TYPE_NAVIGATION = 1
uint8 TYPE_OPERATION = 2

string id # id of the area
string name # name of the area
uint8 type # type of the area. 0 = obstacle, 1 = navigation, 2 = operation
geometry_msgs/PolygonStamped area # area of the zone
