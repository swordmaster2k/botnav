import math

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
        return "x: " + str(self.x) + " y: " + str(self.y)


s_goal = Node(2, 2)

# Sample set of 9 nodes.
nodes = [[Node(0, 0), Node(0, 1), Node(0, 2)], [Node(1, 0), Node(2, 0), Node(2, 1)],
         [Node(1, 1), Node(1, 2), Node(2, 2)]]

# Simple 2x2 grid with every cell cost set to 1.
grid = [[1, 1], [1, 1]]


def h(s):
    # Heuristic based on straight line distance between two points.
    return math.sqrt((s.y - s_goal.y) ** 2 + (s.x - s_goal.x) ** 2)


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
            neighbours.append(None)

    return neighbours


def get_consecutive_neighbours(s):
    consecutive_neighbours = []

    for i in range(len(NEIGHBOUR_DIRECTIONS)):
        try:
            if i == len(NEIGHBOUR_DIRECTIONS) - 1:  # Edge s8 -> s1
                consecutive_neighbours.append(
                    (nodes[s.x + NEIGHBOUR_DIRECTIONS[i][0]][s.y + NEIGHBOUR_DIRECTIONS[i][1]],
                     nodes[s.x + NEIGHBOUR_DIRECTIONS[0][0]][s.y + NEIGHBOUR_DIRECTIONS[0][1]]))
            else:  # All other edges.
                consecutive_neighbours.append(
                    (nodes[s.x + NEIGHBOUR_DIRECTIONS[i][0]][s.y + NEIGHBOUR_DIRECTIONS[i][1]],
                     nodes[s.x + NEIGHBOUR_DIRECTIONS[i + 1][0]][s.y + NEIGHBOUR_DIRECTIONS[i + 1][1]]))

        except IndexError:
            consecutive_neighbours.append(None)

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

    return vs


def update_state(s):
    if s is not s_goal:
        rhs = LARGE

        for edge in get_consecutive_neighbours(s):
            if edge is not None:
                cost = compute_cost(s, edge[0], edge[1])  # s, sa, sb

                if cost < rhs:
                    rhs = cost

        s.rhs = cost

    return None


def compute_shortest_path():
    return None

# Test Field D*.		
s = Node(0, 0)
sa = Node(2, 1)
sb = Node(2, 2)

print(compute_cost(s, sa, sb))
print(get_neighbours(s))
print(get_consecutive_neighbours(s))
print(h(s))