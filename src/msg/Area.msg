uint8 TYPE_EXCLUSION = 0
uint8 TYPE_NAVIGATION = 1
uint8 TYPE_LAWN = 2

string id # id of the zone
string name # name of the zone
uint8 type # type of the area. 0 = obstacle, 1 = navigation, 1 = lawn
geometry_msgs/PolygonStamped area # area of the zone
