import numpy as np
import pygame
import math
from utils import *
from typing import List, Tuple, Optional
import optim
from argparse import ArgumentParser
from robot_kinematics import RoboticsKinematicsSolver


class SerialManipulator:
    '''
    Base class for a serial manipulator. TwoLinkManipulator and ThreeLinkManipulator inherit from this class.'''

    def __init__(self, surface, lengths: np.ndarray, angles: Optional[np.ndarray] = None):
        '''
        Initializes a serial manipulator.

        Parameters
        ----------
        lengths : numpy.ndarray
            The lengths of the links of the manipulator.
        angles : numpy.ndarray, optional
            The angles of the joints of the manipulator. If not specified, the angles are set to 0.
        '''
        self.lengths = lengths
        self.joint_positions = []
        if angles is None:
            self.angles = np.zeros_like(lengths)
        else:
            self.angles = angles
        self.get_joint_positions()
        self.surface = surface
        self.message_surface = pygame.Surface(
            (WIDTH - WORKSPACE_WIDTH, WORKSPACE_HEIGHT))

        pygame.display.update()
        self.message_font = pygame.font.SysFont('Arial', 20)
        self.solver = RoboticsKinematicsSolver()
        self.reset_message_surface()

    def reset_message_surface(self):
        '''
        Resets the message box to the default state.
        '''
        self.message_surface.fill((0, 0, 0))
        text = self.message_font.render('Message Box', True, (255, 255, 255))
        self.message_surface.blit(text, (0, 0))
        pygame.display.update()

    def get_joint_positions(self):
        '''
        Solves the forward kinematics problem to calculate the positions of the joints of the manipulator.
        '''
        self.joint_positions = [np.array([0, 0])]
        for i in range(len(self.lengths)):
            x = self.joint_positions[i][0] + \
                self.lengths[i] * math.cos(sum(self.angles[:i+1]))
            y = self.joint_positions[i][1] + \
                self.lengths[i] * math.sin(sum(self.angles[:i+1]))
            self.joint_positions.append(np.array([x, y]))

    def set_angles(self, angles: np.ndarray):
        '''
        Sets the angles of the joints of the manipulator to the specified values.

        Parameters
        ----------
        angles : np.ndarray
            The angles of the joints of the manipulator.
        '''
        assert len(angles) == len(
            self.lengths), 'The number of angles must be equal to the number of links.'
        assert all([-math.pi < angle <= math.pi for angle in angles]
                   ), 'The angles must be in the interval (-pi/ pi], got {}'.format(angles)
        self.angles = angles
        self.get_joint_positions()

    def draw(self):
        '''
        Draws the manipulator on the input surface.

        Parameters
        ----------
        surface : pygame.Surface
            The surface on which the manipulator is drawn.
        '''
        reset(self.surface, WORKSPACE_WIDTH, WORKSPACE_HEIGHT)

        for i in range(len(self.joint_positions) - 1):
            pygame.draw.line(self.surface, ARM_COLOR, robot_to_canvas(self.joint_positions[i], WORKSPACE_WIDTH, WORKSPACE_HEIGHT), robot_to_canvas(
                self.joint_positions[i + 1], WORKSPACE_WIDTH, WORKSPACE_HEIGHT), 3)

        pygame.draw.circle(self.surface, END_EFFECTOR_COLOR, robot_to_canvas(
            self.joint_positions[-1], WORKSPACE_WIDTH, WORKSPACE_HEIGHT), 10)
        pygame.draw.circle(self.surface, END_EFFECTOR_BORDER_COLOR, robot_to_canvas(
            self.joint_positions[-1], WORKSPACE_WIDTH, WORKSPACE_HEIGHT), 10, 3)
        # Update self.surface
        display_surface.blit(self.surface, (0, 0))
        pygame.display.update()


class TwoLinkManipulator(SerialManipulator):
    def __init__(self, surface, lengths: np.ndarray, angles: Optional[np.ndarray] = None):
        assert len(
            lengths) == 2, 'This is a 2-link manipulator; the number of links must be 2.'
        assert angles is None or len(
            angles) == 2, 'This is a 2-link manipulator; the number of angles must be 2.'
        super().__init__(surface, lengths, angles)
        self.lengths = lengths
        self.joint_positions = []
        if angles is None:
            self.set_angles(np.zeros_like(lengths))
        else:
            self.set_angles(angles)

    def get_joint_angles(self, target: np.ndarray):
        '''
            Solves the inverse kinematics problem analytically to calculate the angles of the joints of the manipulator.

            Parameters
            ----------
            target : np.ndarray
                The position of the end effector of the manipulator.

            Returns
            -------
            list[list]
                Both possible solutions to the inverse kinematics problem.
                Returns None if there  are no solutions.

        '''
        x, y = target
        l1, l2 = self.lengths

        theta = math.atan2(y, x)
        d = math.sqrt(x**2 + y**2)
        cos_alpha = (d**2 + l1**2 - l2**2) / (2 * d * l1)
        if abs(cos_alpha) > 1:
            return None
        alpha = math.acos(cos_alpha)

        theta1_1 = theta - alpha
        theta2_1 = theta + alpha

        cos_beta = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        if abs(cos_beta) > 1:
            return None
        beta = math.acos(cos_beta)

        theta1_2 = math.pi - beta
        theta2_2 = -math.pi + beta

        theta1 = [theta1_1, theta1_2]
        theta2 = [theta2_1, theta2_2]

        return [theta1, theta2]


class ThreeLinkManipulator(SerialManipulator):
    def __init__(self, surface, lengths: List[float], angles: Optional[List[float]] = None):
        '''
        Initializes a 3-link manipulator.

        Parameters
        ----------
        lengths : list
            The lengths of the links of the manipulator.
        angles : list, optional
            The angles of the joints of the manipulator. If None, the angles are set to 0.

        '''
        assert len(
            lengths) == 3, 'This is a 3-link manipulator; the number of links must be 3.'
        assert angles is None or len(
            angles) == 3, 'This is a 3-link manipulator; the number of angles must be 3.'

        super().__init__(surface, lengths, angles)

    def move_to(self, target: np.ndarray, num_points: int = 21, wait_time: int = 250, allow_negative_y: bool = False):
        '''
        Moves the end effector of the manipulator to the target position linearly using num_points equally spaced intermediate points.

        Parameters
        ----------
        target : np.ndarray
            The target position of the end effector.
        num_points : int
            The number of intermediate points to use.
        '''
        font = pygame.font.SysFont('Arial', 20)
        self.reset_message_surface()
        text = font.render(
            'Target Point : (' + str(target[0]) + ',' + str(target[1]) + ')', True, (255, 255, 255))
        self.message_surface.blit(text, (0, 30))

        if np.linalg.norm(target) > CIRCLE_RADIUS:
            text = font.render('Target out of workspace',
                               True, (255, 255, 255))
            self.message_surface.blit(text, (0, 60))
            return None

        if not allow_negative_y:
            if target[1] < 0:
                font = pygame.font.SysFont('Arial', 20)
                text = font.render(
                    'Target out of bounds! Targets below y-axis are not allowed!', True, (255, 255, 255))
                self.message_surface.blit(text, (0, 60))

                return None

        # Get intermediate points
        x = np.linspace(self.joint_positions[-1][0], target[0], num_points)
        y = np.linspace(self.joint_positions[-1][1], target[1], num_points)
        points = [(x[i], y[i]) for i in range(num_points)]

        # Move end effector to each intermediate point
        self.table = []
        for i, point in enumerate(points):
            self.get_joint_angles(point)
            if i == 0:
                table_row = [i+1, tuple([round(k, 2) for k in self.joint_positions[1]]), map_angle(self.angles[0]), 'N/A', tuple([round(k, 2)
                                                                                                                                  for k in self.joint_positions[2]]), self.angles[1], 'N/A', tuple([round(k, 2) for k in self.joint_positions[3]]), self.angles[2], 'N/A', 'N/A']
            else:
                prev_angles = table_row[2], table_row[5], table_row[8]
                table_row = [i+1,
                             tuple([round(k, 2) for k in self.joint_positions[1]]), map_angle(
                                 self.angles[0]),
                             abs(map_angle(self.angles[0]) - prev_angles[0]),
                             tuple([round(k, 2)
                                   for k in self.joint_positions[2]]), self.angles[1],
                             abs(self.angles[1] - prev_angles[1]),
                             tuple([round(k, 2)
                                   for k in self.joint_positions[3]]), self.angles[2],
                             abs(self.angles[2] - prev_angles[2]),
                             round(max(abs(map_angle(self.angles[0]) - prev_angles[0]),
                                       abs(self.angles[1] - prev_angles[1]),
                                       abs(self.angles[2] - prev_angles[2])), 3)]

            # Keep only the first 3 decimal places
            table_row = [round(x, 3) if isinstance(
                x, float) else x for x in table_row]
            self.table.append(table_row)
            self.draw()
            pygame.time.delay(wait_time)

        self.draw()
        table.data = self.table
        table.draw()

    def get_joint_angles(self, target: np.ndarray):

        def cost(x: float):
            '''
            Cost function to minimize. This function sets the first joint angle (A-B) to x and solves the inverse kinematics problem for the 2-link manipulator formed by B-C and C-D.

            This is used to obtain the solution to the inverse kinematics problem for the 3-link manipulator. This function returns the maximum change in any joint angle between the current and previous states.  

            Parameters
            ----------
            x : float
                The angle of the first joint (A-B)
            '''

            two_link_manipulator = TwoLinkManipulator(
                workspace_surface, self.lengths[1:])
            joint_position = np.array(
                [self.lengths[0]*math.cos(x), self.lengths[0]*math.sin(x)])
            relative_target = target - joint_position

            sol = two_link_manipulator.get_joint_angles(relative_target)
            if sol is None:
                return {'cost': 10000, 'point': None}

            sol1, sol2 = sol
            sol1_theta_1_prime, sol1_theta_2_prime = sol1
            sol2_theta_1_prime, sol2_theta_2_prime = sol2

            sol1_theta_1 = sol1_theta_1_prime - x
            sol2_theta_1 = sol2_theta_1_prime - x

            sol1_theta_2 = sol1_theta_2_prime
            sol2_theta_2 = sol2_theta_2_prime

            sol1_3lm = np.array([x, sol1_theta_1, sol1_theta_2])
            sol2_3lm = np.array([x, sol2_theta_1, sol2_theta_2])

            sol1_diff = np.abs(sol1_3lm - self.angles)
            sol2_diff = np.abs(sol2_3lm - self.angles)

            sol1_cost = np.max(sol1_diff)
            sol2_cost = np.max(sol2_diff)

            joint_positions_sol1 = self.solver.solve_fkp(
                self.lengths, sol1_3lm)
            joint_positions_sol2 = self.solver.solve_fkp(
                self.lengths, sol2_3lm)

            if joint_positions_sol1[2][1] < 0:
                if joint_positions_sol2[2][1] < 0:
                    return {'cost': 10000, 'point': None}
                else:
                    return {'cost': sol2_cost, 'point': sol2_3lm}

            if joint_positions_sol2[2][1] < 0:
                if joint_positions_sol1[2][1] < 0:
                    return {'cost': 10000, 'point': None}
                else:
                    return {'cost': sol1_cost, 'point': sol1_3lm}

            if sol1_cost <= sol2_cost:
                return {'cost': sol1_cost, 'point': sol1_3lm}
            else:
                return {'cost': sol2_cost, 'point': sol2_3lm}

        # Minimize the cost function
        sweep_range = [0, math.pi]

        if args.optimization_algorithm == 'box-search':
            res = optim.minimize(cost, sweep_range, args.num_points)
        elif args.optimization_algorithm == 'gradient-descent':
            res = optim.gradient_descent(
                cost, [self.angles[0], -self.angles[0]], alpha=3e-3)
        min_cost = res[1]
        if min_cost > 1000:
            # Display error message to self.surface
            self.reset_message_surface()
            text = self.message_font.render(
                'Clicked point is outside workspace!', True, (255, 255, 255))
            self.message_surface.blit(text, (0, 60))
            pygame.display.update()
            return None

        best_angles = res[2]
        best_angles = [domainize(angle) for angle in best_angles]

        # theta1_cap = domainize(res)

        self.set_angles(best_angles)

        self.draw()


class Table:
    def __init__(self, surface, data: list = []):
        self.data = data
        self.font = pygame.font.SysFont('Arial', 14)
        self.font.set_bold(True)
        self.header = ['Dot #', 'B pos', 'ang(A-B)', 'del(A-B)', 'C pos', 'ang(B-C)',
                       'del(B-C)', 'D-pos', 'ang(C-D)', 'del(C-D)', 'Max \u0394 \u03B8']
        self.header_color = (0, 0, 0)
        self.row_color = (0, 0, 0)
        self.border_color = (0, 0, 0)
        self.border_width = 2
        self.surface = surface
        self.surface.fill((255, 255, 255))
        self.spaces = [60, 120, 60, 60, 120, 60, 60, 120, 60, 60, 60]
        self.header_spaces = [80, 90, 60, 90, 90, 60, 100, 80, 60, 60, 60]

    def draw(self):
        self.surface.fill((255, 255, 255))

        for i, header in enumerate(self.header):
            text = self.font.render(header, True, self.header_color)
            self.surface.blit(text, (3 + sum(self.header_spaces[:i]), 0))

        # Draw the rows
        for i, row in enumerate(self.data):
            for j, cell in enumerate(row):
                text = self.font.render(str(cell), True, self.row_color)
                self.surface.blit(text, (3 + sum(self.spaces[:j]), (i+1)*15))

        # Draw the border
        pygame.draw.rect(self.surface, self.border_color,
                         (0, 0, 1000, 1000), self.border_width)

        pygame.display.update()


if __name__ == '__main__':
    pygame.init()
    pygame.display.set_caption('Serial Manipulator Simulator')
    clock = pygame.time.Clock()

    parser = ArgumentParser()
    parser.add_argument('--allow_negative_y', action='store_true')
    parser.add_argument('--optimization_algorithm', type=str, default='box-search', choices=[
                        'box-search', 'gradient-descent'], help='The optimization algorithm to use for inverse kinematics. Gradient descent requires tuning the learning rate.')

    parser.add_argument('--wait_time', type=int, default=250,
                        help='Time to display images for between successive points in the trajectory.')
    parser.add_argument('--num_points', type=int, default=100,
                        help='Number of points to use for box-search. Only specify if using box-search.')
    args = parser.parse_args()

    WIDTH = pygame.display.Info().current_w*0.9
    HEIGHT = pygame.display.Info().current_h*0.9

    BACKGROUND_COLOR = (255, 255, 255)
    GRID_COLOR = (128, 128, 128)
    CIRCLE_COLOR = (0, 0, 255)
    ARM_COLOR = (255, 0, 0)
    END_EFFECTOR_COLOR = (255, 255, 0)
    END_EFFECTOR_BORDER_COLOR = (0, 0, 0)
    CIRCLE_X, CIRCLE_Y = 300, 300
    CIRCLE_RADIUS = 300
    CIRCLE_THICKNESS = 1
    WORKSPACE_WIDTH = 2*CIRCLE_RADIUS
    WORKSPACE_HEIGHT = CIRCLE_RADIUS

    # Pygame Setup
    display_surface = pygame.display.set_mode((WIDTH, HEIGHT))
    workspace_surface = pygame.Surface((WORKSPACE_WIDTH, WORKSPACE_HEIGHT))
    workspace_surface.fill(BACKGROUND_COLOR)

    # Create a table
    table_width = WIDTH
    table_height = HEIGHT - WORKSPACE_HEIGHT - 10

    table_surface = pygame.Surface((table_width, table_height))
    table_surface.fill((255, 0, 0))

    table = Table(table_surface)
    table.draw()

    robot = ThreeLinkManipulator(workspace_surface, [150, 100, 50], [
                                 math.pi/2, -math.pi/2, -math.pi/2])
    robot.draw()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            elif event.type == pygame.MOUSEBUTTONUP:
                robot.reset_message_surface()
                robot.move_to(canvas_to_robot(pygame.mouse.get_pos(), WORKSPACE_WIDTH, WORKSPACE_HEIGHT),
                              num_points=21, wait_time=args.wait_time, allow_negative_y=args.allow_negative_y)

        display_surface.blit(robot.message_surface, (WORKSPACE_WIDTH, 0))
        display_surface.blit(workspace_surface, (0, 0))
        display_surface.blit(table_surface, (0, WORKSPACE_HEIGHT + 10))
        pygame.display.update()

        clock.tick(60)
