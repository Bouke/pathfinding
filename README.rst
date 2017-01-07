Note: this is a dump of some experimental code. Use only for your own experiments.

A* pathfinding algorithm in Python using OSM data.

Instructions:

* Download an XML export of subset of OSM data (start with small maps; e.g. 5x5 km)::

    wget -O map.osm.xml http://overpass-api.de/api/map?bbox=6.0305,52.4907,6.1199,52.5559
* Set start/dest to OSM node id, or use look-up by street name.
