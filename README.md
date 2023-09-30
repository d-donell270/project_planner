# project_planner
WRITTEN IN DSSL2, a close cousin of Python, with a similar indentation-sensitive syntax and similar function and keyword names when possible. IDE- DrRacket.

I implemented a trip planning API (Application Programming Interface) that provides routing and searching services.
The trip planner stores map data representing three kinds of items and answer three kinds of queries about them. The following two subsections describe the items and the queries.

1.1 ITEMS
• A position has a latitude and a longitude, both numbers.
• A road segment has two endpoints, both positions.
• A point-of-interest (POI) has a position, a category (a string), and a name (a string). The name of a point-of-interest is unique across all points-of-interest, but a category may be shared by multiple points-of-interest. Each
position can feature zero, one, or more POIs.
There three assumptions about segments and positions:
1. All roads are two-way roads.
2. The length of a road segment is the standard Euclidian distance(x^2 + y^2) between its endpoints.
Points-of-interest can only be found at a road segment endpoint.

1.2 QUERIES
The trip planner supports three forms of queries:
locate-all: Takes a point-of-interest category; returns the positions of all points- of-interest in the given category. The positions can returned be in any order you want, but the result should not include duplicates.
plan-route: Takes a starting position (latitude and longitude) and the name of a point-of-interest; returns a shortest path from the starting position to the named point-of-interest. You can assume the starting position is at a road segment endpoint. Returns the empty list if the destination does not exist or is unreachable.
find-nearby: Takes a starting position (latitude and longitude), a point-of- interest category, and a limit n; returns the (up to) n points-of-interest in the given category nearest the starting position. You can assume the starting position is at a road segment endpoint.
