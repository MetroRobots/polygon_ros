from polygon_utils import make_point
from polygon_utils.shortest_path import shortest_path
from polygon_msgs.msg import Polygon2D, Polygon2DCollection


def test_direct():
    polys = Polygon2DCollection()

    start = make_point(-3.0, -3.0)
    goal = make_point(3.0, 3.0)

    shortest = shortest_path(polys, start, goal)
    assert len(shortest) == 2
    assert shortest[0] == start
    assert shortest[-1] == goal


def test_basic():
    triangle = Polygon2D()
    triangle.points.append(make_point(-2.0, 0.0))
    triangle.points.append(make_point(2.0, 1.0))
    triangle.points.append(make_point(1.0, -1.0))

    triangle2 = Polygon2D()
    triangle2.points.append(make_point(-2.0, 0.0))
    triangle2.points.append(make_point(-2.0, 2.0))
    triangle2.points.append(make_point(-4.0, 0.0))

    polys = Polygon2DCollection()
    polys.polygons.append(triangle)
    polys.polygons.append(triangle2)

    start = make_point(-3.0, -3.0)
    goal = make_point(3.0, 3.0)

    shortest = shortest_path(polys, start, goal)
    assert len(shortest) == 4
    assert shortest[0] == start
    assert shortest[-1] == goal


def test_overlap():
    rect0 = Polygon2D()
    rect0.points.append(make_point(0.2, -8.5))
    rect0.points.append(make_point(-6.2, -8.5))
    rect0.points.append(make_point(-6.2, -7.9))
    rect0.points.append(make_point(0.2, -7.9))

    rect1 = Polygon2D()
    rect1.points.append(make_point(0.2, -8.0))
    rect1.points.append(make_point(6.0, -12.0))
    rect1.points.append(make_point(5.6, -12.4))
    rect1.points.append(make_point(-0.1, -8.3))

    polys = Polygon2DCollection()
    polys.polygons.append(rect0)
    polys.polygons.append(rect1)

    start = make_point(0.0, -15.0)
    goal = make_point(0.0, -5.0)

    shortest = shortest_path(polys, start, goal)
    assert len(shortest) == 4
    assert shortest[0] == start
    assert shortest[-1] == goal
