from polygon_msgs.msg import Point2D


def make_point(x, y):
    msg = Point2D()
    msg.x = x
    msg.y = y
    return msg
