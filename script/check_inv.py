from sympy.interactive.printing import init_printing
init_printing(use_unicode=True, wrap_line=True)
import sympy

[fx, fy, cx, cy] = sympy.symbols(['fx','fy','cx','cy'])
K = sympy.matrices.Matrix([[fx, 0, cx],[0, fy, cy],[0, 0 , 1]])
P = sympy.matrices.Matrix([[fx, 0, cx, 0],[0, fy, cy, 0],[0, 0 , 1, 0], [0,0,0,1]])
print(K.inv())
print(P.inv())