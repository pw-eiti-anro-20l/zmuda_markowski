#! /usr/bin/python

import json
from tf.transformations import *

if __name__ == '__main__': 
    x, z = (1, 0, 0), (0, 0, 1)  # wektory dla osi w notacji dh potrzebne do macierzy rotacji
    parameters = {}
    with open("../dh.json", "r") as file:
        parameters = json.loads(file.read())

    with open('../urdf_val.yaml', 'w') as file:
        for key in parameters.keys():
            a, d, alfa, theta = parameters[key]
            alfa  = float(alfa)
	    a  = float(a) 
            d = float(d)
	    theta = float(theta)
            trans_z = translation_matrix((0, 0, d))  # ustalenie macierzy jednorodnej dla parametrow
            rot_z = rotation_matrix(theta, z)
            trans_x = translation_matrix((a, 0, 0))
            rot_x = rotation_matrix(alfa, x)
            matrix = concatenate_matrices(trans_z, rot_z, trans_x, rot_x)

            rpy = euler_from_matrix(matrix)  # konwersja z dh
            xyz = translation_from_matrix(matrix)

            file.write(key + ":\n")
            file.write("  j_xyz: {} {} {}\n".format(*xyz))
            file.write("  j_rpy: {} {} {}\n".format(*rpy))
            file.write("  l_xyz: {} 0 0\n".format(xyz[0] / 2))
            file.write("  l_rpy: 0 0 0\n")
            file.write("  l_len: {}\n".format(a))
