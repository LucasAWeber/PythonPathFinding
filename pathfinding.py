# ShockingRotom August 2021
# click left mouse button for starting point
# click left mouse button for ending point
# click/hold left mouse button to draw boundaries
# click right mouse button to erase boundaries
# press space/return to start pathfinding
# press c to clear the screen and start over (only usable when not currently pathfinding)
# press esc to exit pathfinding

import pygame
import numpy as np
import heapq
from warnings import warn

pygame.init()

# Colours
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PURPLE = (255, 0, 255)

# Important Vars
COUNT = 15
SIZE = 50

size = (SIZE * COUNT, SIZE * COUNT)
screen = pygame.display.set_mode(size, 0, 32)

screen.fill(BLACK)

pygame.display.set_caption("Path Finding")

clock = pygame.time.Clock()


class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        return self.f < other.f

    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        return self.f > other.f


# Functions
# Creates a matrix of the board
def create_matrix(x):
    matrix = np.zeros((x, x))
    return matrix


# Draws matrix
def draw_matrix(screen, count, SIZE, array):
    for r in range(count):
        for c in range(count):
            if array[r][c] == 0:
                colour = BLACK
            elif array[r][c] == 1:
                colour = WHITE
            elif array[r][c] == 2:
                colour = GREEN
            elif array[r][c] == 3:
                colour = RED
            elif array[r][c] == 4:
                colour = BLUE
            else:
                colour = PURPLE

            pygame.draw.rect(screen, colour, (r * SIZE, c * SIZE, SIZE, SIZE))


# Draws highlight
def draw_highlight(screen, SIZE):
    c = int(pygame.mouse.get_pos()[1] / SIZE) * SIZE
    r = int(pygame.mouse.get_pos()[0] / SIZE) * SIZE
    pos = (r, c)
    select = pygame.Surface((SIZE, SIZE))
    # Makes square semi transparent
    select.set_alpha(128)
    select.fill(WHITE)
    screen.blit(select, pos)


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement=False):
    """
    Returns a list of tuples as a path from the given start to the given end in the given maze
    :param maze:
    :param start:
    :param end:
    :return:
    """

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze[0]) * len(maze) // 2)

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1

        # if outer_iterations > max_iterations:
            # if we hit this point return the path such as it is
            # it will not contain the destination
            # warn("giving up on pathfinding too many iterations")
            # return return_path(current_node)

            # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        for i in closed_list:
            maze[i.position] = 4

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []

        for new_position in adjacent_squares:  # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                    len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == 1:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            maze[child.position] = 5
            for r in range(COUNT):
                for c in range(COUNT):
                    if maze[r][c] == 5:
                        try:
                            if maze[r - 1][c] != 0 and maze[r + 1][c] != 0 and maze[r][c - 1] != 0 and maze[r][c + 1] != 0:
                                maze[r][c] = 4
                        except:
                            pass

            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                        (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if
                    child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

        draw_matrix(screen, COUNT, SIZE, maze)

        pygame.display.flip()

        clock.tick(10)

    warn("Couldn't get a path to destination")
    return None


matrix = create_matrix(COUNT)
draw_matrix(screen, COUNT, SIZE, matrix)
pygame.display.flip()

running = True
start = True
end = True
drawing = False
pathfinding = False
r = 0
c = 0
start_pt = (0, 0)
end_pt = (0, 0)

while running:
    # User Events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_SPACE or event.key == pygame.K_RETURN:
                pathfinding = True
            elif event.key == pygame.K_c:
                matrix = create_matrix(COUNT)
                start = True
                end = True
        elif event.type == pygame.MOUSEBUTTONDOWN and pygame.mouse.get_pressed(3)[0]:
            if start:
                start = False
                colour = 2
                start_pt = (r, c)
            elif end:
                end = False
                colour = 3
                end_pt = (r, c)
            else:
                drawing = True
                if matrix[r][c] == 0:
                    colour = 1
                else:
                    colour = matrix[r][c]
            matrix[r][c] = colour
        elif event.type == pygame.MOUSEBUTTONDOWN and pygame.mouse.get_pressed(3)[2]:
            if not start and not end:
                if matrix[r][c] == 1:
                    matrix[r][c] = 0
        elif event.type == pygame.MOUSEBUTTONUP and not pygame.mouse.get_pressed(3)[0]:
            if drawing:
                drawing = False

    if pathfinding:
        matrix_copy = matrix.copy()

        for i in astar(matrix_copy, start_pt, end_pt):
            matrix[i] = 2

        draw_matrix(screen, COUNT, SIZE, matrix)

        pygame.display.flip()
        pathfinding = False

    c = int(pygame.mouse.get_pos()[1] / SIZE)
    r = int(pygame.mouse.get_pos()[0] / SIZE)

    if drawing:
        if matrix[r][c] == 0:
            matrix[r][c] = 1

    draw_matrix(screen, COUNT, SIZE, matrix)
    draw_highlight(screen, SIZE)

    pygame.display.flip()
