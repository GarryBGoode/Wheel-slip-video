import numpy as np
from manim import *
from scipy.optimize import root
from scipy.special import factorial
import bezier as bz

class Hatch_lines(VGroup):
    def __init__(self, target_mobject: Mobject, angle=PI/6, offset=0.3,**kwargs):
        super().__init__(**kwargs)
        self.target = target_mobject
        target_size_xy = [self.target.get_critical_point(RIGHT)-self.target.get_critical_point(LEFT),
                       self.target.get_critical_point(UP)-self.target.get_critical_point(DOWN)]
        target_size_diag = np.linalg.norm(target_size_xy)
        num_lines = int(target_size_diag//offset)
        line_v = np.array([np.cos(angle),np.sin(angle),0])
        offs_v = np.array([-np.sin(angle),np.cos(angle),0])
        center = self.target.get_center()
        for k in range(num_lines*2):
            line_loc = Line(center-line_v*target_size_diag*1.5,center+line_v*target_size_diag*1.5)
            line_loc.shift((k-(num_lines-1))*offs_v*offset)
            # line_loc = Intersection(line_loc,self.target)
            intersect_idx = curve_intersection(self.target,line_loc)
            if any(intersect_idx[1]):

                intersect_indexes = sorted(intersect_idx[1])
                for j in range(len(intersect_idx[1])//2):
                    idx1 = int(intersect_indexes[0+2*j] // 1)
                    alpha1 =   intersect_indexes[0+2*j] % 1
                    idx2 = int(intersect_indexes[1+2*j] // 1)
                    alpha2 =   intersect_indexes[1+2*j] % 1
                    point1 = line_loc.get_nth_curve_function(idx1)(alpha1)
                    point2 = line_loc.get_nth_curve_function(idx2)(alpha2)
                    line_loc2 = Line(start=point1,end=point2,**kwargs)
                    self.add(line_loc2)


def curve_intersection(vmob1: VMobject, vmob2: VMobject, num_initial_guesses=4):
    """intersection points of 2 curves"""

    intersect_indx_1 = np.array([])
    intersect_indx_2 = np.array([])
    for i in range(vmob1.get_num_curves()):
        for j in range(vmob2.get_num_curves()):
            points_1 = vmob1.get_nth_curve_points(i)
            points_2 = vmob2.get_nth_curve_points(j)
            x_range_1 = np.array([np.amax(points_1[:, 0]), np.amin(points_1[:, 0])])
            x_range_2 = np.array([np.amax(points_2[:, 0]), np.amin(points_2[:, 0])])
            y_range_1 = np.array([np.amax(points_1[:, 1]), np.amin(points_1[:, 1])])
            y_range_2 = np.array([np.amax(points_2[:, 1]), np.amin(points_2[:, 1])])

            distinct_x = x_range_2[1] > x_range_1[0] or x_range_1[1] > x_range_2[0]
            distinct_y = y_range_2[1] > y_range_1[0] or y_range_1[1] > y_range_2[0]

            overlap = not (distinct_x or distinct_y)

            if overlap:

                curve_fun_1 = [np.polynomial.Polynomial(bezier_points_to_poly(points_1[:,0])),
                               np.polynomial.Polynomial(bezier_points_to_poly(points_1[:,1]))]
                curve_fun_2 = [np.polynomial.Polynomial(bezier_points_to_poly(points_2[:, 0])),
                               np.polynomial.Polynomial(bezier_points_to_poly(points_2[:, 1]))]
                deriv_1 = [np.polynomial.Polynomial(bezier_points_to_poly(points_1[:,0])).deriv(1),
                           np.polynomial.Polynomial(bezier_points_to_poly(points_1[:,1])).deriv(1)]
                deriv_2 = [np.polynomial.Polynomial(bezier_points_to_poly(points_2[:, 0])).deriv(1),
                           np.polynomial.Polynomial(bezier_points_to_poly(points_2[:, 1])).deriv(1)]

                init_guess_t = np.linspace(1/(num_initial_guesses+1),
                                           num_initial_guesses/(num_initial_guesses+1),
                                           num_initial_guesses)
                init_guess_t_combined = np.array([[init_guess_t[i],init_guess_t[j]]
                                                  for i in range(num_initial_guesses)
                                                  for j in range(num_initial_guesses)])

                def target_func(t):
                    return [poly_pair[0](t[0]) - poly_pair[1](t[1]) for poly_pair in zip(curve_fun_1, curve_fun_2)]
                def jacobian_func(t):
                    return [[poly_pair[0](t[0]), - poly_pair[1](t[1])] for poly_pair in zip(deriv_1, deriv_2)]

                for guess in init_guess_t_combined:
                    sol = root(target_func, guess, method='hybr')
                    if sol.success:
                        if 0 < sol.x[0] < 1 and 0 < sol.x[1] < 1:
                            # if no intersections have been found yet
                            if len(intersect_indx_1)==0:
                                intersect_indx_1 = np.append(intersect_indx_1, sol.x[0] + i)
                                intersect_indx_2 = np.append(intersect_indx_2, sol.x[1] + j)
                            else:
                                diffs= zip(intersect_indx_1-(sol.x[0] + i),intersect_indx_2 - (sol.x[1] + j))
                                diff_eval = [np.abs(v[0])<1e-6 and np.abs(v[1])<1e-6 for v in diffs]
                                if not any(diff_eval):
                                    intersect_indx_1 = np.append(intersect_indx_1, sol.x[0] + i)
                                    intersect_indx_2 = np.append(intersect_indx_2, sol.x[1] + j)

    return intersect_indx_1, intersect_indx_2


def bezier_points_to_poly(points):
    n = len(points) - 1
    C = [choose(n,j) * sum([(-1.0) ** (i + j) * points[i] * choose(j,i) for i in range(j+1)]) for j in range(n + 1)]
    return C
