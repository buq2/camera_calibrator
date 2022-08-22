from sympy.interactive.printing import init_printing
init_printing(use_unicode=True, wrap_line=True)
import sympy

# Find a projection of a point 'p' on to a plane (4 vector), ax + by + cy + d = 0
# such that projection is done using direction vector 'dir'.
#
# Used in ProjectToPlane when arg projection_direction is given

# (p-dir*t).dot(plane.block<3,1>(0,0)) + plane(3) == 0
#  (p(0)-dir(0)*t)*plane(0)
# +(p(1)-dir(1)*t)*plane(1)
# +(p(2)-dir(2)*t)*plane(2)
# +plane(3)
# = 0

[p0, p1, p2, dir0, dir1, dir2, t, plane0, plane1, plane2, plane3] = sympy.symbols(['p0', 'p1', 'p2', 'dir0', 'dir1', 'dir2', 't', 'plane0', 'plane1', 'plane2', 'plane3'])

eq = (p0-dir0*t)*plane0 + (p1-dir1*t)*plane1 + (p2-dir2*t)*plane2 + plane3

print(sympy.solve(eq, t))