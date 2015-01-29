import math

PATH = []
OPEN = []
LARGE = 10000000000
SQRT_2 = 1.414213562
NEIGHBOUR_DIRECTIONS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1,), (1, -1)]


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = LARGE
        self.rhs = LARGE
        self.key = [None, None]

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " g: " + str(self.g) + " rhs: " + str(self.rhs)


s_goal = Node(0, 0)
s_start = Node(2, 2)

# Sample set of 9 nodes.
nodes = [[s_goal, Node(0, 1), Node(0, 2)], [Node(1, 0), Node(1, 1), Node(1, 2)],
         [Node(2, 0), Node(2, 1), s_start]]

# Simple 2x2 grid with every cell cost set to 1.
grid = [[1, 1], [1, 1]]


def h(s):
    # Heuristic based on straight line distance between two points.
    return round(math.sqrt((s.y - s_goal.y) ** 2 + (s.x - s_goal.x) ** 2), 3)


def key(s):
    if s.g < s.rhs:
        s.key[0] = s.g + h(s)
        s.key[1] = s.g
    else:
        s.key[0] = s.rhs + h(s)
        s.key[1] = s.rhs


def get_neighbours(s):
    neighbours = []

    for direction in NEIGHBOUR_DIRECTIONS:
        try:
            neighbours.append(nodes[s.x + direction[0]][s.y + direction[1]])
        except IndexError:
            pass

    return neighbours


def get_consecutive_neighbours(s):
    consecutive_neighbours = []

    for i in range(len(NEIGHBOUR_DIRECTIONS)):
        try:
            if i == len(NEIGHBOUR_DIRECTIONS) - 1:  # Edge s8 -> s1
                x1 = s.x + NEIGHBOUR_DIRECTIONS[i][0]
                y1 = s.y + NEIGHBOUR_DIRECTIONS[i][1]
                x2 = s.x + NEIGHBOUR_DIRECTIONS[0][0]
                y2 = s.y + NEIGHBOUR_DIRECTIONS[0][1]

                if x1 > -1 and y1 > -1 and x2 > -1 and y2 > -1:
                    consecutive_neighbours.append(
                        (nodes[s.x + NEIGHBOUR_DIRECTIONS[i][0]][s.y + NEIGHBOUR_DIRECTIONS[i][1]],
                         nodes[s.x + NEIGHBOUR_DIRECTIONS[0][0]][s.y + NEIGHBOUR_DIRECTIONS[0][1]]))
            else:  # All other edges.
                x1 = s.x + NEIGHBOUR_DIRECTIONS[i][0]
                y1 = s.y + NEIGHBOUR_DIRECTIONS[i][1]
                x2 = s.x + NEIGHBOUR_DIRECTIONS[i + 1][0]
                y2 = s.y + NEIGHBOUR_DIRECTIONS[i + 1][1]

                if x1 > -1 and y1 > -1 and x2 > -1 and y2 > -1:
                    consecutive_neighbours.append(
                        (nodes[s.x + NEIGHBOUR_DIRECTIONS[i][0]][s.y + NEIGHBOUR_DIRECTIONS[i][1]],
                         nodes[s.x + NEIGHBOUR_DIRECTIONS[i + 1][0]][s.y + NEIGHBOUR_DIRECTIONS[i + 1][1]]))

        except IndexError:
            pass
        except AttributeError:
            pass

    return consecutive_neighbours


def is_diagonal_neighbour(s, sa):
    x = s.x - sa.x
    y = s.y - sa.y

    if x == 1 or x == -1:
        if y == 1 or y == -1:
            return True

    return False


def get_cell_cost(x, y):
    try:
        return grid[x][y]
    except IndexError:
        return LARGE


def compute_cost(s, sa, sb):
    if is_diagonal_neighbour(s, sa):
        s1 = sb
        s2 = sa
    else:
        s1 = sa
        s2 = sb

    # Get mid point of diagonal neighbours s and s2.
    mid_x = (s.x + s2.x) / 2
    mid_y = (s.y + s2.y) / 2

    # Map mid_x and mid_y to a cell cost, if the x and y is out of
    # bounds c == LARGE.
    c = get_cell_cost(math.floor(mid_x), math.floor(mid_y))

    # Get the difference between (s, s2) and (s, s1) mid-points.
    difference_x = mid_x - ((s.x + s1.x) / 2)
    difference_y = mid_y - ((s.y + s1.y) / 2)

    if s.x == s1.x:
        if difference_x > 0:
            # b is the traversal cost of the cell to the left.
            b = get_cell_cost(math.floor(mid_x) - 1, math.floor(mid_y))
        else:
            # b is the traversal cost of the cell to the right.
            b = get_cell_cost(math.floor(mid_x) + 1, math.floor(mid_y) - 1)
    elif s.y == s1.y:
        if difference_y > 0:
            # b is the traversal cost of the cell immediately below.
            b = get_cell_cost(math.floor(mid_x), math.floor(mid_y) - 1)
        else:
            # b is the traversal cost of the cell immediately above.
            b = get_cell_cost(math.floor(mid_x), math.floor(mid_y) + 1)
    else:
        b = LARGE

    if min(c, b) == LARGE:
        vs = LARGE
    elif s1.g <= s2.g:
        vs = min(c, b) + s1.g
    else:
        f = s1.g - s2.g

        if f <= b:
            if c <= f:
                vs = c * SQRT_2 + s2.g
            else:
                y = min(f / (math.sqrt(c ** 2 - f ** 2)), 1)

                vs = c * math.sqrt(1 + y ** 2) + f * (1 - y) + s2.g
        else:
            if c <= b:
                vs = c * SQRT_2 + s2.g
            else:
                x = 1 - min(b / (math.sqrt(c ** 2 - b ** 2)), 1)

                vs = c * math.sqrt(1 + ((1 - x) ** 2)) + (b * x) + s2.g

    if vs > LARGE:
        vs = LARGE

    return round(vs, 3)


def update_state(s):
    if s is not s_goal:
        rhs = s.rhs

        for edge in get_consecutive_neighbours(s):
            cost = compute_cost(s, edge[0], edge[1])  # s, sa, sb

            if cost < rhs:
                rhs = cost

        s.rhs = rhs

    if OPEN.count(s) > 0:
        OPEN.remove(s)

    if s.g != s.rhs:
        key(s)
        if len(OPEN) == 0:
            OPEN.append(s)
        else:
            for i in range(len(OPEN)):
                if s.key[0] <= OPEN[i].key[0] and s.key[1] <= OPEN[i].key[1]:
                    OPEN.insert(i, s)
                    return

            OPEN.append(s)


def compute_shortest_path():
    while (OPEN[0].key[0] < s_start.key[0] or OPEN[0].key[1] < s_start.key[1]) or s_start.rhs != s_start.g:
        s = OPEN.pop()

        if s.g > s.rhs:
            s.g = s.rhs

            for neighbour in get_neighbours(s):
                if neighbour is not s_goal:
                    update_state(neighbour)
        else:
            s.g = LARGE

            for neighbour in get_neighbours(s):
                if neighbour is not s_goal:
                    update_state(neighbour)

        if len(OPEN) == 0:
            break

    for column in nodes:
        for node in column:
            print(str(node))

    # Extract the path.
    while s_start.x != s_goal.y or s_start.y != s_goal.y:
        consecutive_neighbours = get_consecutive_neighbours(Node(math.floor(s_start.x), math.floor(s_start.y)))

        if len(consecutive_neighbours) > 0:
            lowest_edge_cost = LARGE
            lowest_pair = None

            for edge in consecutive_neighbours:
                if edge[0].g + edge[1].g < lowest_edge_cost:
                    lowest_edge_cost = edge[0].g + edge[1].g
                    lowest_pair = edge

        # Always ensure that s1 has the highest g-value.
        if lowest_pair[1].g < lowest_pair[0].g:
            s1 = lowest_pair[1]
            s2 = lowest_pair[0]
        else:
            s1 = lowest_pair[0]
            s2 = lowest_pair[1]

        f = s1.g - s2.g  # f = g(s1) - g(s2)

        x_difference = s1.x - s2.x  # s1.x - s2

        if x_difference != 0:
            if x_difference > 0:
                f = -f

            s_start.x = s1.x + f
            s_start.y = s1.y
        else:
            y_difference = s1.y - s2.y  # s1.y - s2.y

            if y_difference > 0:
                f = -f

            s_start.x = s1.x
            s_start.y = s2.y + f

            PATH.append((s_start.x, s_start.y))


def main():
    key(s_start)

    s_goal.g = LARGE
    s_goal.rhs = 0
    key(s_goal)
    s_goal.key[1] = 0

    OPEN.append(s_goal)

    compute_shortest_path()

    print('\n')

    for point in PATH:
        print(point)


main()