import math
import numpy as np
import pygame
from typing import Tuple
# Helper functions for the project


def robot_to_canvas(point, width, height):
    '''
    Converts a point from the Cartesian coordinate system to the Pygame coordinate system.

    Parameters
    ----------
    point : tuple
        A point in the Cartesian coordinate system.
    width : int
        The width of the Pygame display surface.
    height : int
        The height of the Pygame display surface.

    Returns
    -------
    tuple
        A point in the Pygame coordinate system.
    '''

    return (point[0] + width/2, height - point[1])


def canvas_to_robot(point, width, height):
    '''
    Converts a point from the Pygame coordinate system to the Cartesian coordinate system.

    Parameters
    ----------
    point : tuple
        A point in the Pygame coordinate system.
    width : int
        The width of the Pygame display surface.
    height : int
        The height of the Pygame display surface.

    Returns
    -------
    tuple
        A point in the Cartesian coordinate system.
    '''

    return (point[0] - width/2, height - point[1])


def draw_grid(surface: pygame.Surface, width: int, height: int, grid_color: Tuple[int, int, int] = (128, 128, 128)):
    for x in range(-width, width, 20):
        pygame.draw.line(surface, grid_color, robot_to_canvas(
            (x, 0), width, height), robot_to_canvas((x, height), width, height))
    for y in range(0, height, 20):
        pygame.draw.line(surface, grid_color, robot_to_canvas(
            (-300, y), width, height), robot_to_canvas((300, y), width, height))
    pygame.display.update()


def draw_circle(surface: pygame.Surface, width: int, center: Tuple[float, float], color: Tuple[int, int, int] = (0, 0, 255), radius: int = 300):
    '''
    Draws a circle on the given surface with the given parameters.
    '''
    pygame.draw.circle(surface, color=color, center=center,
                       radius=radius, width=width)
    pygame.display.update()


def reset(surface: pygame.Surface, width: int, height: int,  background_color: Tuple[int, int, int] = (255, 255, 255), grid_color: Tuple[int, int, int] = (128, 128, 128), color: Tuple[int, int, int] = (0, 0, 255), radius: int = 300, circle_center: Tuple[float, float] = None, circle_thickness: int = 1):
    if circle_center is None:
        circle_center = (width/2, height)
    surface.fill(background_color)
    draw_grid(surface, width, height, grid_color)
    draw_circle(surface, width=circle_thickness,
                center=circle_center, color=color, radius=radius)


def map_angle(theta: float):
    '''
    Converts angle measured from the x-axis to the angle measured from the y-axis.
    '''
    return math.pi/2 - theta


def domainize(theta: float):
    '''
    Maps the angle to the domain [-pi, pi].
    '''
    return (theta + math.pi) % (2*math.pi) - math.pi


class Table:
    def __init__(self, screen, pos, rows, cols, cell_width, cell_height, border_width=1, border_color=(0, 0, 0), font_size=20, font_color=(0, 0, 0)):
        self.screen = screen
        self.pos = pos
        self.rows = rows
        self.cols = cols
        self.cell_width = cell_width
        self.cell_height = cell_height
        self.border_width = border_width
        self.border_color = border_color
        self.font_size = font_size
        self.font_color = font_color
        self.font = pygame.font.Font(None, self.font_size)
        self.cells = []

        for row in range(self.rows):
            row_cells = []
            for col in range(self.cols):
                cell_rect = pygame.Rect(
                    self.pos[0] + col * self.cell_width, self.pos[1] + row * self.cell_height, self.cell_width, self.cell_height)
                row_cells.append(cell_rect)
            self.cells.append(row_cells)

    def draw(self):
        for row in range(self.rows):
            for col in range(self.cols):
                cell_rect = self.cells[row][col]
                pygame.draw.rect(self.screen, self.border_color,
                                 cell_rect, self.border_width)

                text = self.font.render(f"{row}, {col}", True, self.font_color)
                text_rect = text.get_rect(center=cell_rect.center)
                self.screen.blit(text, text_rect)

        pygame.display.update()

    def get_cell_rect(self, row, col):
        return self.cells[row][col]
