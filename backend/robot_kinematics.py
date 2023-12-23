import numpy as np
import math
import optim
from utils import *
from typing import List


class RoboticsKinematicsSolver:
    def solve_fkp(self, lengths, angles):
        '''
        Solves the forward kinematics problem to calculate the positions of the joints of the manipulator.

        Parameters
        ----------
        lengths : list
            The lengths of the links of the manipulator.
        angles : list
            The angles of the joints of the manipulator.

        Returns
        -------
        list
            The positions of the joints of the manipulator.
        '''
        if angles is None:
            return "Unreachable"
        joint_positions = [(0, 0)]
        for i in range(len(lengths)):
            x = joint_positions[i][0] + lengths[i] * \
                math.cos(sum(angles[:i+1]))
            y = joint_positions[i][1] + lengths[i] * \
                math.sin(sum(angles[:i+1]))
            joint_positions.append((x, y))
        return joint_positions

    def solve_ikp(self, link_lengths: np.ndarray, target: np.ndarray, curr_angles: np.ndarray = None) -> np.ndarray:
        '''
        Solves the inverse kinematics problem for a 2-link or 3-link serial manipulator. Solution for 2 link manipulator is analytic while solution for 3-link manipulator is obtained through numerical optimization

        Args:
            link_lengths: Lengths of the links of the serial manipulator
            target: Target point to be reached by the serial manipulator
            curr_angles: For 3-link manipulator, current angle is needed

        Returns: 
            angles: The optimal angles for the serial manipulator to reach the target
        '''

        if len(link_lengths) == 2:
            return self._solve_two_link_ikp(link_lengths, target)
        elif len(link_lengths) == 3:
            if curr_angles is None:
                raise ValueError(
                    "curr_angles not specified for 3-link manipulator")
            return self._solve_three_link_ikp(link_lengths, curr_angles, target)
        else:
            raise ValueError(
                f"Expected len(link_lengths) to be 2 or 3, but got {len(link_lengths)}")

    def _solve_two_link_ikp(self, link_lengths: np.ndarray, target):
        '''
            Solves the Inverse Kinematics Problem (IKP) for a two-link serial manipulator analytically to calculate the angles of the joints of the manipulator that would move the manipulator to a specified target position.

            Parameters
            ----------
            link_lengths: Lenghts of the links of the 2d serial manipulator
            target : np.ndarray
                The position of the end effector of the manipulator.

            Returns
            -------
            list[list]
                Both possible solutions to the inverse kinematics problem.
                Returns None if there  are no solutions.

        '''
        x, y = target
        l1, l2 = link_lengths

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

    def _solve_three_link_ikp(self, link_lengths, curr_angles, target, optimization_algorithm="box-search", num_points=10):

        def cost(x: float):
            '''
            Cost function to minimize. This function sets the first joint angle (A-B) to x and solves the inverse kinematics problem for the 2-link manipulator formed by B-C and C-D.

            This is used to obtain the solution to the inverse kinematics problem for the 3-link manipulator. This function returns the maximum change in any joint angle between the current and previous states.  

            Parameters
            ----------
            x : float
                The angle of the first joint (A-B)
            '''

            joint_position = np.array(
                [link_lengths[0]*math.cos(x), link_lengths[0]*math.sin(x)])
            relative_target = target - joint_position

            sol = self._solve_two_link_ikp(link_lengths[1:], relative_target)
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

            sol1_diff = np.abs(sol1_3lm - curr_angles)
            sol2_diff = np.abs(sol2_3lm - curr_angles)

            sol1_cost = np.max(sol1_diff)
            sol2_cost = np.max(sol2_diff)

            joint_positions_sol1 = self.solve_fkp(
                link_lengths, sol1_3lm)
            joint_positions_sol2 = self.solve_fkp(
                link_lengths, sol2_3lm)

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
        if optimization_algorithm == 'box-search':
            res = optim.minimize(cost, sweep_range, num_points)
        elif optimization_algorithm == 'gradient-descent':
            res = optim.gradient_descent(
                cost, [self.angles[0], -self.angles[0]], alpha=3e-3)
        min_cost = res[1]
        if min_cost > 1000:
            return None

        best_angles = res[2]
        best_angles = [domainize(angle) for angle in best_angles]
        # theta1_cap = domainize(res)
        return best_angles
