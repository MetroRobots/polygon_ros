from .shapely_lib import polygon_from_msg, point_from_msg, point_to_msg
from shapely.geometry import LineString, Point
from shapely.ops import unary_union
import heapq


class VisGraph:
    def __init__(self):
        self.vertices = set()
        self.visibility = {}
        self.mp = None

    def build(self, polygons, verbose=False):
        for polygon in polygons:
            rings = [polygon.exterior] + list(polygon.interiors)
            for ring in rings:
                for pt in ring.coords:
                    self.vertices.add(pt)
        # Use unary_union instead of MultiPolygon to merge overlapping polygons
        self.mp = unary_union(polygons)

        for v0 in self.vertices:
            self.visibility[v0] = {}

            for v1 in self.vertices:
                if v0 == v1:
                    continue
                line = LineString([v0, v1])
                if not self.mp.crosses(line):
                    self.visibility[v0][v1] = line.length

    def shortest_path(self, start, goal):
        direct = LineString([start, goal])
        if not self.mp.crosses(direct):
            return [Point(*p) for p in direct.coords]
        start_t = start.x, start.y
        goal_t = goal.x, goal.y

        queue = []
        goals = {}
        for v in self.vertices:
            sline = LineString([start, v])
            gline = LineString([v, goal])

            if not self.mp.crosses(sline):
                heapq.heappush(queue, (sline.length, [start_t, v]))
            if not self.mp.crosses(gline):
                goals[v] = gline.length

        while queue:
            d, path = heapq.heappop(queue)
            v0 = path[-1]
            if v0 == goal_t:
                return [Point(*p) for p in path]

            if v0 in goals:
                new_d = d + goals[v0]
                new_path = path + [goal_t]
                heapq.heappush(queue, (new_d, new_path))

            for v1, d2 in self.visibility[v0].items():
                if v1 in path:
                    continue
                new_d = d + d2
                new_path = path + [v1]
                heapq.heappush(queue, (new_d, new_path))


def graph_from_msg(polygons_msg, verbose=False):
    g = VisGraph()
    g.build([polygon_from_msg(poly) for poly in polygons_msg.polygons], verbose=verbose)
    return g


def shortest_path_from_graph(graph, start, goal):
    vg_path = graph.shortest_path(point_from_msg(start), point_from_msg(goal))
    return [point_to_msg(pt) for pt in vg_path]


def shortest_path(polygons, start, goal):
    g = graph_from_msg(polygons)
    return shortest_path_from_graph(g, start, goal)
