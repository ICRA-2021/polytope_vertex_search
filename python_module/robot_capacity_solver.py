#!/usr/bin/env python
from math import sin, cos 
import numpy as np
import numpy.matlib 
import itertools

# minkowski sum
from scipy.spatial import ConvexHull, Delaunay


# velocity manipulability calculation
def manipulability_velocity(Jacobian_position, dq_max):
    # jacobian calculation
    Jac = Jacobian_position
    # limits scaling
    W = np.diagflat(dq_max)
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # return the singular values and the unit vector angle
    return [S, U]

# force manipulability calculation
def manipulability_force(Jacobian_position, t_max):
    # jacobian calculation
    Jac = Jacobian_position
    # limits scaling
    W = np.linalg.pinv(np.diagflat(t_max))
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # return the singular values and the unit vector angle
    return [np.divide(1,S), U]

# maximal end effector force
def force_polytope_intersection_auctus(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min):

    # jacobian calculation
    Jac =  np.hstack((Jacobian1,Jacobian2))
    t_min = np.vstack((t1_min,t2_min))
    t_max = np.vstack((t1_max,t1_max))

    return force_polytope_auctus(Jac, t_max,t_min)

# maximal end effector force
def force_polytope_sum_auctus(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min):

    f_vertex1, t_vertex1, gravity1 = force_polytope_auctus(Jacobian1, t1_max, t1_min)
    f_vertex2, t_vertex2, gravity2 = force_polytope_auctus(Jacobian2, t2_max, t2_min)
    m, n = Jacobian1.shape
    f_sum = np.zeros((f_vertex1.shape[1]*f_vertex2.shape[1],m))
    for i in range(f_vertex1.shape[1]):
        for j in range(f_vertex2.shape[1]):
            f_sum[i*f_vertex2.shape[1] + j] = np.array(f_vertex1[:,i]+f_vertex2[:,j]).flat

    hull = ConvexHull(f_sum, qhull_options='QJ')
    f_vertex = np.array(f_sum[hull.vertices]).T

    polytope_faces = []
    for face in hull.simplices:
        polytope_faces.append(np.array(f_sum[face]).T)

    return f_vertex, polytope_faces

# maximal end effector force
def force_polytope_auctus(Jacobian, t_max, t_min, gravity = None):

    # jacobian calculation
    Jac = Jacobian
    m, n = Jac.shape

    # if gravity not specified
    if gravity == None:
        gravity = np.zeros((n,1))

    # calculate svd
    U, S, V = np.linalg.svd(Jac)
    r = np.linalg.matrix_rank(Jac)
    V1 = np.array(V.transpose()[:,:m])
    V2 = np.array(V.transpose()[:,m:])

    # jacobian matrix pseudo inverse
    J_n_invT = np.linalg.pinv(Jac.transpose())

    # matrix of axis vectors - for polytope search
    T_vec = np.diagflat(t_max-t_min)
    T_min = np.matlib.repmat(t_min - gravity,1,2**m)
    t_max_g = t_max - gravity

    fixed_vectors_combinations = np.array(list(itertools.combinations(range(n),m)))
    permutations = np.array(list(itertools.product([0, 1], repeat=m))).T

    f_vertex = []
    t_vertex = []
    # A loop to go through all pairs of adjacent vertices
    for fixed_vectors in fixed_vectors_combinations:
        # find all n-m face vector indexes
        face_vectors = np.delete(range(n), fixed_vectors) 

        S = T_min.copy()
        for i in range(m): S[fixed_vectors[i], permutations[i] > 0] = t_max_g[fixed_vectors[i]]
        S_v2 = V2.transpose().dot(S)

        # vectors to be used
        T = T_vec[:,face_vectors]
        # solve the linear system for the edge vectors tl and tj
        Z = V2.transpose().dot(-T)

        # figure out if some solutions can be discarded
        Z_min = Z.copy()
        Z_min[Z_min > 0] = 0
        Z_max = Z.copy()
        Z_max[Z_max < 0] = 0
        S_min = Z_min.dot(np.ones((n-m,1)))
        S_max = Z_max.dot(np.ones((n-m,1)))
        to_reject = np.any(S_v2 - S_min < - 10**-7, axis=0) + np.any(S_v2 - S_max > 10**-7, axis=0)
        if np.all(to_reject): # all should be discarded
            continue
        S = S[:,~to_reject]
        S_v2 = V2.transpose().dot(S)
        
        # check the rank and invert
        Z_inv = np.linalg.pinv(Z)
                                
        # calculate the forces for each face
        X = Z_inv.dot(S_v2)
        # check if inverse correct - all error 0
        t_err = np.any( abs(S_v2 - Z.dot(X)) > 10**-7, axis=0) 
        # remove the solutions that are not in polytope 
        to_remove = (np.any(X < -10**-7, axis=0) + np.any(X - 1 > 10**-7 , axis=0)) + t_err
        X= X[:, ~to_remove]
        S= S[:, ~to_remove]
        if t_vertex == []:
            t_vertex =  S+T.dot(X)
        # add vertex torque
        t_vertex = np.hstack((t_vertex, S+T.dot(X)))

    t_vertex = make_unique(t_vertex)
    # calculate the forces based on the vertex torques
    f_vertex = J_n_invT.dot( t_vertex )
    return f_vertex, t_vertex, gravity
    
def force_polytope_ordered(Jacobian, t_max, t_min, gravity = None):
    force_vertex, t_vertex, gravity = force_polytope_auctus(Jacobian, t_max, t_min, gravity)
    m, n = Jacobian.shape
    
    polytope_faces = []
    if force_vertex.shape[0] == 1:
        polytope_faces.append(force_vertex)
    elif force_vertex.shape[0] == 2:
        polytope_faces.append(force_vertex[:, order_index(force_vertex)])
    else:        
        for i in range(n):
            fi = np.array(force_vertex[:,np.isclose(t_vertex[i,:], (t_min[i] - gravity[i]),1e-5)])
            if fi != []:
                if fi.shape[1] > 3:
                    polytope_faces.append(fi[:,order_index(make_2d(fi))])
                else: 
                    polytope_faces.append(fi)
            fi = np.array(force_vertex[:,np.isclose(t_vertex[i,:], (t_max[i] - gravity[i]),1e-5)])
            if fi != []:
                if fi.shape[1] > 3:
                    polytope_faces.append(fi[:,order_index(make_2d(fi))])
                else: 
                    polytope_faces.append(fi)
    return [force_vertex, polytope_faces]

def force_polytope_intersection_ordered(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min):
    force_vertex, t_vertex, gravity = force_polytope_intersection_auctus(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min)
    m, n = Jacobian1.shape
    t_max_int = np.vstack((t1_max,t2_max))
    t_min_int = np.vstack((t1_min,t2_min))

    polytopes = []
    for i in range(2*n):
        fi = np.array(force_vertex[:,np.isclose(t_vertex[i,:], (t_min_int[i] - gravity[i]),1e-5)])
        if fi != []:
            if fi.shape[1] > 3:
                polytopes.append(fi[:,order_index(make_2d(fi))])
            else: 
                polytopes.append(fi)
        fi = np.array(force_vertex[:,np.isclose(t_vertex[i,:], (t_max_int[i] - gravity[i]),1e-5)])
        if fi != []:
            if fi.shape[1] > 3:
                polytopes.append(fi[:,order_index(make_2d(fi))])
            else: 
                polytopes.append(fi)
    return [force_vertex, polytopes]


def make_2d(points):
    U = points[:,1]-points[:,0]   # define 1st ortogonal vector
    for i in range(2, points.shape[1]): # find 2nd ortogonal vector (avoid collinear)
        V = points[:,i]-points[:,0]
        if abs(abs(U.dot(V)) - np.linalg.norm(U)*np.linalg.norm(V)) > 10**-7:
            break
    
    U = U / np.linalg.norm(U)
    V = V / np.linalg.norm(V)

    W = np.cross(U,V) # this will be near zero if A,B,C are on single line
    U = np.cross(V,W)
    
    x = U.dot(points)
    y = V.dot(points)

    return np.array([x, y])

def order_index(points):
    px = np.array(points[0,:]).ravel()
    py = np.array(points[1,:]).ravel()
    p_mean = np.array(np.mean(points,axis=1)).ravel()

    angles = np.arctan2( (py-p_mean[1]), (px-p_mean[0]))
    sort_index = np.argsort(angles)
    return sort_index

def make_unique(points):
    unique_points = np.unique(np.around(points,7), axis=1)
    return unique_points
         
# definition of the four_link_solver module
if __name__ == '__main__':
    robot_capacity_solver() 