import argparse, os, itertools
from typing import List, Tuple
from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString, orient
from shapely.geometry import mapping
from Dijkstra import Dijkstra

def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    robot = [(0, -r), (r, 0), (0, r), (-r, 0)]
    obstacle = []
    original_shape = orient(original_shape, sign=1.0)
    for i in mapping(original_shape)['coordinates']:
        for x in i:
            obstacle.append(x)

    if (len(obstacle)%2==1):
        obstacle.pop()
    min_vertices = []
    r_vertices_num = 4
    obstacle_vertices_num = len(obstacle)
    i = 0; j = 0

    robot.append(robot[0])
    robot.append(robot[1])
    robot.append(robot[2])
    robot.append(robot[3])

    obstacle.append(obstacle[0])
    obstacle.append(obstacle[1])
    obstacle.append(obstacle[2])
    obstacle.append(obstacle[3])
    while(True):
        if (i>obstacle_vertices_num and j>r_vertices_num):
            break

        new_vertex = (obstacle[i][0]+robot[j][0],obstacle[i][1]+robot[j][1])
        min_vertices.append(new_vertex)
        robotX = robot[(j+1)][0]-robot[j][0]
        robotY = robot[j+1][1]-robot[j][1]
        obstacleX = obstacle[i+1][0] - obstacle[i][0]
        obstacleY = obstacle[i+1][1] - obstacle[i][1]
        angle_decider = obstacleX*robotY-obstacleY*robotX

        if (angle_decider>0):
            i += 1
        elif (angle_decider<0):
            j += 1
        else:
            i += 1
            j += 1
    
    min_vertices.pop()
    min_vertices.pop()
    return Polygon(min_vertices)

def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    vertices = []; lines = []
    for poly in obstacles:
        for i in mapping(poly)['coordinates']:
            for x in i:
                vertices.append(x)

    # adding the start and goal nodes to the graph
    if source is not None:
        vertices.append(source)
    if dest is not None:
        vertices.append(dest)

    for pair in itertools.combinations(vertices, 2):
        intersects = False
        line = LineString(pair)
        for poly in obstacles:
            if (line.covered_by(poly) or line.crosses(poly)):
                intersects = True
                break
        if not intersects:
            lines.append(line)
    return lines

def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)

def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()
    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)
    plotter1.show_graph()

    # step 2:
    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()
    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)
    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))
    lines = get_visibility_graph(c_space_obstacles, source, dest)
    shortest_path, cost = Dijkstra(lines, source, dest)

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))
    plotter3.show_graph()