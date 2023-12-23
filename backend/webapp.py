from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import requests
import numpy as np
from robot_kinematics import RoboticsKinematicsSolver
import math
from utils import canvas_to_robot, robot_to_canvas

roboticsKinematicsSolver = RoboticsKinematicsSolver()

link_lengths = [150, 100, 50]
thetas = [math.pi/2, -math.pi/2, -math.pi/2]


app = Flask(__name__)
CORS(app, origins="http://localhost:5173")


@app.route('/')
def home():
    return "Healthy!"


@app.route("/resetManipulator", methods=["POST"])
def resetManipulator():
    global thetas, link_lengths
    data = request.get_json()
    width = data.get("screen_width")
    height = data.get("screen_height")
    init_thetas = [math.pi/2, -math.pi/2, -math.pi/2]
    joint_positions = roboticsKinematicsSolver.solve_fkp(
        link_lengths, init_thetas)
    print("Joint Positions: ", joint_positions)
    thetas = init_thetas

    joint_positions = [robot_to_canvas(
        joint_position, width, height) for joint_position in joint_positions]

    response_data = {"joint_positions": joint_positions}
    response = jsonify(response_data)
    response.headers.add('Access-Control-Allow-Origin',
                         'http://localhost:5173')

    return response


@app.route("/getJointPositions", methods=["POST"])
def getJointPositions():
    global thetas, link_lengths
    data = request.get_json()
    width = data.get("screen_width")
    height = data.get("screen_height")

    target = data.get('target')
    print('target_canvas: ', target)
    target = np.array(canvas_to_robot(target, width, height))
    print('target_robot: ', target)

    print("target: ", target)

    curr_end_eff_loc = np.array(roboticsKinematicsSolver.solve_fkp(
        link_lengths, thetas)[-1])

    print('Curr End Eff: ', curr_end_eff_loc)
    diff = target - curr_end_eff_loc
    print('Diff: ', diff)
    num_points = 20

    joint_positions = []
    thetas_new = thetas
    for i in range(1, num_points + 1):
        curr_target = curr_end_eff_loc + i*diff/num_points
        print("curr_target: ", curr_target)
        thetas_new = roboticsKinematicsSolver.solve_ikp(
            link_lengths, curr_target, thetas_new)

        if not thetas_new:
            print("Unreachable")
            response_data = {"status": "Unreachable"}
            break

        curr_joint_positions = roboticsKinematicsSolver.solve_fkp(
            link_lengths, thetas_new)
        curr_joint_positions = [robot_to_canvas(
            curr_joint_position, width, height) for curr_joint_position in curr_joint_positions]

        joint_positions.append(curr_joint_positions)

        response_data = {"joint_positions": joint_positions,
                         "status": "Reachable"}

    if thetas_new:
        thetas = thetas_new

    response = jsonify(response_data)
    response.headers.add('Access-Control-Allow-Origin',
                         'http://localhost:5173')

    return response


if __name__ == '__main__':
    app.run(debug=True)
