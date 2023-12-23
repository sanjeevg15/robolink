import numpy as np
from typing import List


def minimize(cost, sweep_range, num_points):
    '''
    Evaluates the cost function at a number of points in a given range and returns the point with the lowest cost. 

    Parameters
    ----------
    cost : function
        Cost function
    sweep_range : list
        Range of values to evaluate the cost function at
    num_points : int
        Number of points to evaluate the cost function at

    Returns
    -------
    min_point : float
        Point with the lowest cost
    min_cost : float
        Lowest cost
    best_angles : list
        Angles corresponding to the lowest cost
    '''
    # Initialize the minimum cost and the corresponding point
    min_cost = np.inf
    min_point = None
    best_angles = None

    # Evaluate the cost function at a number of points in the given range
    for i in range(num_points):

        point = sweep_range[0] + i*(sweep_range[1] - sweep_range[0])/num_points
        res = cost(point)
        cost_at_point = res['cost']
        angles = res['point']

        if cost_at_point < min_cost:
            min_cost = cost_at_point
            min_point = point
            best_angles = angles
    return min_point, min_cost, best_angles


def gradient(cost, point, delta=1e-4):
    '''
    Calculates the gradient of a cost function at a given point.

    Parameters
    ----------
    cost : function
        Cost function
    point : float
        Point at which to calculate the gradient
    delta : float
        Step size to use when calculating the gradient

    Returns
    -------
    grad : float
        Gradient of the cost function at the given point
    '''
    grad = (cost(point + delta)['cost'] -
            cost(point - delta)['cost'])/(2*delta)
    return grad


def gradient_descent(cost, x0: List, alpha=1e-4, max_iters=100, tol=1e-4):
    '''
    Performs gradient descent to find the minimum of a cost function.

    Parameters
    ----------
    cost : function
        Cost function
    x0 : List[float]
        List of initial points
    alpha : float
        Learning rate
    max_iters : int
        Maximum number of iterations

    Returns
    -------
    x_min : float
        Point with the lowest cost
    min_cost : float
        Lowest cost
    '''
    # Initialize placeholders for minimum cost and the corresponding point
    min_cost = np.inf
    x_min = None

    res = cost(x0[0])
    best_angles = res['point']

    for x in x0:
        # Perform gradient descent
        for _ in range(max_iters):
            grad = gradient(cost, x)
            x -= alpha*grad/np.linalg.norm(grad)
            if np.linalg.norm(grad) < tol:
                print('Gradient descent converged')
                break
            res = cost(x)
            cost_at_x = res['cost']
            print(f'x: {x}, cost: {cost_at_x}, grad: {np.linalg.norm(grad)}')

            if cost_at_x < min_cost:
                min_cost = cost_at_x
                x_min = x
                best_angles = res['point']

    return x_min, min_cost, best_angles
