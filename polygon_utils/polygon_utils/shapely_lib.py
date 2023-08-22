from . import make_point
from polygon_msgs.msg import Polygon2D, ComplexPolygon2D

try:
    from shapely.geometry import Point, Polygon
except ImportError:
    import sys
    sys.stderr.write('Installing the shapely library by hand required. '
                     'Please run\n'
                     '\tsudo pip3 install shapely\n')
    exit(-1)


# Convert point

def point_from_msg(point):
    return Point(point.x, point.y)


def point_to_msg(point):
    return make_point(point.x, point.y)


# Convert list of points

def _get_ring(ring):
    return [[pt.x, pt.y] for pt in ring]


def _get_ring_msg(ring):
    return [point_to_msg(point) for point in ring]


# Convert polygons

def polygon_from_msg(polygon):
    if isinstance(polygon, Polygon2D):
        return Polygon(_get_ring(polygon.points))
    else:
        return Polygon(_get_ring(polygon.outer.points), [_get_ring(ring.points) for ring in polygon.inner])


def polygon_to_msg(polygon):
    if polygon.interiors:
        msg = ComplexPolygon2D()
        msg.outer.points = _get_ring_msg(polygon.exterior)
        for interior in polygon.interiors:
            msg.inner.append(_get_ring_msg(interior))
    else:
        msg = Polygon2D()
        msg.points = _get_ring_msg(polygon.exterior)
    return msg
