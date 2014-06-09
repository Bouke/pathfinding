"""
A* pathfinding algorithm in Python using OSM data.

Instructions:

* Set start/dest to OSM node id, or use look-up by street name.
* Download an XML export of subset of OSM data (start with small maps; e.g. 5x5 km)

Note that:

* Algorithm performance is bad.
* Algorithm only takes distance into account.
* Algorithm does not filter road using all OSM tags.

A* algorithm from http://www.policyalmanac.org/games/aStarTutorial.htm
"""

from enum import Enum

from math import sin, radians, cos, atan2, sqrt
from operator import attrgetter
from lxml import etree as ET


class NodeRef(object):
    def __init__(self, elm):
        self.element = elm

    @property
    def id(self):
        return self.element.attrib['ref']

    @property
    def node(self):
        return get_node(self.id)

    def __eq__(self, other):
        if isinstance(other, (Node, NodeRef)):
            return self.id == other.id
        if isinstance(other, VisitedNode):
            return self.id == other.node.id

    def __repr__(self):
        return "<NodeRef node=%(node)s>" % {'node': self.node}


class Node(object):
    def __init__(self, elm):
        self.element = elm

    @property
    def id(self):
        return self.element.attrib['id']

    @property
    def lat(self):
        return float(self.element.attrib['lat'])

    @property
    def lon(self):
        return float(self.element.attrib['lon'])

    def __eq__(self, other):
        return self.id == other.id

    def __repr__(self):
        return "<Node id=%(id)s, lat=%(lat)s, lon=%(lon)s>" % {
            'id': self.id,
            'lat': self.lat,
            'lon': self.lon,
        }


class VisitedNode(object):
    def __init__(self, node, parent=None, g=None, h=None):
        self.node = node
        self.parent = parent
        self.g = g
        self.h = h

    @property
    def f(self):
        return self.g + self.h if self.g is not None and self.h is not None else None

    def __eq__(self, other):
        if isinstance(other, VisitedNode):
            return self.node.id == other.node.id
        if isinstance(other, (Node, NodeRef)):
            return self.node.id == other.id

    def __gt__(self, other):
        return self.f > other.f

    def __repr__(self):
        return "<VisitedNode node=%(node)s, g=%(g).3f, h=%(h).3f>" % {
            'node': repr(self.node),
            'f': self.f,
            'g': self.g,
            'h': self.h,
        }


class Way(object):
    def __init__(self, elm):
        self.element = elm

    @property
    def id(self):
        return self.element.attrib['id']

    @property
    def name(self):
        tag = self.element.find('tag[@k="name"]')
        return tag.attrib['v'] if tag is not None else None

    @property
    def highway(self):
        return self.get_tag('highway')

    @property
    def nodes(self):
        return [NodeRef(e) for e in self.element.findall('nd')]

    @property
    def oneway(self):
        return self.get_tag("oneway") == "yes"

    @property
    def access(self):
        return self.get_tag('access') or "yes"

    @property
    def vehicle(self):
        return self.get_tag('vehicle') or self.access

    @property
    def motor_vehicle(self):
        return self.get_tag("motor_vehicle") or self.vehicle

    @property
    def motorcar(self):
        return self.get_tag("motorcar") or self.vehicle

    @property
    def motorcyle(self):
        return self.get_tag("motorcyle") or self.vehicle

    def get_tag(self, key):
        tag = self.element.find('tag[@k="%s"]' % key)
        return tag.attrib['v'] if tag is not None else None

    def __eq__(self, other):
        return other is not None and self.name == other.name

    def __repr__(self):
        return "<Way id=%(id)s, name=%(name)s, highway=%(highway)s, oneway=%(oneway)s>" % {
            'id': self.id,
            'name': self.name,
            'highway': self.highway,
            'oneway': self.oneway,
        }


class RouteType(Enum):
    foot = 1
    cycle = 2
    car = 3


class Router(object):
    highways = {
        'motorway': (RouteType.car,),
        'trunk': (RouteType.car,),
        'primary': (RouteType.car,),
        'secondary': (RouteType.car,),
        'tertiary': (RouteType.car,),
        'unclassified': (RouteType.car,),
        'residential': (RouteType.car,),
        'service': (RouteType.car,),
    }

    def __init__(self, route_type, from_, to):
        assert route_type == RouteType.car

        self.route_type = route_type
        self.from_ = VisitedNode(from_)
        self.to = to

    def route(self):
        self.from_.g = 0
        self.from_.h = self.distance(self.from_.node, self.to)

        self.visited = []
        self.stack = [self.from_]
        self.nodes = []

        while len(self.stack) > 0:
            self.stack.sort(key=attrgetter('f'))
            selected = self.stack.pop(0)
            self.visited.append(selected)

            print("Exploring %(node_id)s (%(distance).03f km to go, %(visited)s nodes seen, %(roads)s roads left)" % {
                'node_id': repr(selected.node.id),
                'distance': selected.h,
                'roads': len(self.stack),
                'visited': len(self.visited),
            })

            self.explore(selected)

            if selected.node == dest:
                break
        else:
            return False

        while selected:
            self.nodes.insert(0, selected)
            selected = selected.parent

        return self.nodes

    def explore(self, selected):
        for way in get_ways(selected.node):
            # print(" [w] %s" % repr(way))

            nodes = way.nodes

            # Filter the nodes adjacent to the selected node
            index = nodes.index(selected.node)
            path_nodes = nodes[index + 1:index + 2]
            if index > 0:
                path_nodes += nodes[index - 1:index]

            for ref in path_nodes:
                if not self.is_way_section_accessible(way, selected.node, ref.node):
                    continue

                g = selected.g + self.distance(selected.node, ref.node)

                if ref in self.visited:
                    continue

                if ref in self.stack:
                    other = [n for n in self.stack if n == ref.node][0]
                    if other.g > g:
                        self.stack.remove(other)
                    else:
                        continue

                self.stack.append(VisitedNode(ref.node, parent=selected, g=g, h=self.distance(ref.node, self.to)))
                # print("    √ %s" % ref.node)

    def is_way_section_accessible(self, way, from_, to):
        if way.oneway and way.nodes.index(from_) > way.nodes.index(to):
            print("    ｘ %(way)s is one-way" % {'way': way.name})
            return False

        if self.route_type not in self.highways.get(way.highway, ()):
            print("    x %(way)s is not accessible (type: '%(highway)s')" % {
                'way': way.name,
                'highway': way.highway,
            })
            return False

        if self.route_type == RouteType.car and way.motorcar == 'no':
            print("    ｘ %(way)s is not accessible (motorcar: %(access)s)" % {
                'way': way.name,
                'access': way.motorcar,
            })
            return False

        return True

    def distance(self, from_, to):
        return distance((from_.lat, from_.lon), (to.lat, to.lon))


def distance(p1, p2):
    R = 6373  # radius of the earth
    dlat = radians(p2[0]) - radians(p1[0])
    dlon = radians(p2[1]) - radians(p1[1])
    a = (sin(dlat / 2)) ** 2 + cos(radians(p1[0])) * cos(radians(p2[0])) * (sin(dlon / 2)) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


class RouteInstructions(object):
    def __init__(self, nodes):
        self.nodes = nodes

    def __iter__(self):
        yield "Route from %(from)s to %(to)s (%(distance)0.2f km)" % {
            'from': get_ways(self.nodes[0].node)[0].name,
            'to': get_ways(self.nodes[-1].node)[0].name,
            'distance': self.nodes[-1].g,
        }

        dist = 0
        prev_way = None
        prev_node = router.nodes[0]
        for node in router.nodes[1:]:
            dist += distance((prev_node.node.lat, prev_node.node.lon), (node.node.lat, node.node.lon))

            way = get_ways(prev_node.node, node.node)[0]

            if way != prev_way:
                if prev_way is None:
                    yield "Start on %s" % way.name
                else:
                    yield "After %(distance)0.2f continue on %(way)s" % {
                        'distance': dist,
                        'way': way.name,
                    }
                dist = 0

            prev_node = node
            prev_way = way

        yield "After %(distance)0.2f arrive on %(way)s" % {
            'distance': dist,
            'way': prev_way.name,
        }

    def __str__(self):
        return '\n'.join(iter(self))


def get_node(id_):
    elm = root.find('node[@id="%s"]' % id_)
    if elm is None:
        raise IndexError("No node found with id %s" % id_)
    return Node(elm)


def get_ways(*nodes):
    ways = root.xpath('way[tag/@k="highway" %s]' % ' '.join([' and nd/@ref=%s' % n.id for n in nodes]))
    return [Way(w) for w in ways]


root = ET.parse("map.osm.xml").getroot()

def find_way(name):
    ways = root.xpath('way[tag[@k="name" and @v="%s"]]' % name)
    return [Way(w) for w in ways]

start = find_way("Uiterwaard")[0].nodes[0].node
dest = find_way("Hasselterbrug")[0].nodes[0].node

print(start, dest)

router = Router(RouteType.car, start, dest)

if not router.route():
    router.visited.sort(key=attrgetter('h'))
    print("Could not calculate route, closest node was %(node)s; near %(near)s" % {
        'node': router.visited[0],
        'near': ' and '.join([str(w.name) for w in get_ways(router.visited[0].node)]),
    })
    exit(-1)

for instruction in RouteInstructions(router.nodes):
    print(instruction)
