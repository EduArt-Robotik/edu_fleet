from sympy import Matrix, Symbol, MatrixSymbol, cos, sin, init_printing, latex, simplify

def export(left_side : str, right_side_sage_element) -> str:
  print(left_side + ' = \n' + str(right_side_sage_element))
  return left_side + ' = \n' + latex(right_side_sage_element) + '\n\n'

def do_formula() -> str:
  latex_export_string: str = str()

  # prediction moving equations
  dt = Symbol('dt')

  ## acceleration of robot is constant
  ## in robot coordinate system
  a_x_t1 = Symbol('a_{x_{t-1}}')
  a_y_t1 = Symbol('a_{y_{t-1}}')
  a_t1 = Matrix([[a_x_t1], [a_y_t1]])
  a_t = a_t1

  latex_export_string += export('\\textbf{a}_{t-1}', a_t1)
  latex_export_string += export('\\textbf{a}_t', a_t)

  ## velocity
  ## in robot coordinate system
  v_x_t1 = Symbol('v_{x_{t-1}}')
  v_y_t1 = Symbol('v_{y_{t-1}}')

  v_t1 = Matrix([[v_x_t1], [v_y_t1]])
  v_t = v_t1 + a_t1 * dt

  latex_export_string += export('\\textbf{v}_{t-1}', v_t1)
  latex_export_string += export('\\textbf{v}_t', v_t)

  ## yaw
  ## in world coordinate system
  yaw_t = Symbol('\\phi_z')
  yaw_t1 = Symbol('\\phi_{z_{t-1}}')
  yaw_rate_t1 = Symbol('\\phi_{z_{t-1}}\\frac{d}{dt}')
  yaw_rate_t = yaw_rate_t1
  yaw_t = yaw_t1 + yaw_rate_t1 * dt

  latex_export_string += export('\\phi_{z_t}', yaw_t)
  latex_export_string += export('\\phi_{z_t}\\frac{d}{dt}', yaw_rate_t)

  ## position
  ## in world coordinate system
  R_t1 = Matrix([
    [cos(yaw_t1), -sin(yaw_t1)],
    [sin(yaw_t1),  cos(yaw_t1)]
  ])
  p_x_t1 = Symbol('p_x_(t-1)')
  p_y_t1 = Symbol('p_y_(t-1)')
  p_t1 = Matrix([[p_x_t1], [p_y_t1]])
  
  p_t = p_t1 + R_t1 * v_t1 * dt + 1.0/2.0 * R_t1 * Matrix([[a_x_t1**2], [a_y_t1**2]]) * dt

  latex_export_string += export('\\textbf{R}_{t-1}', R_t1)
  latex_export_string += export('\\textbf{p}_{t-1}', p_t1)
  latex_export_string += export('\\textbf{p}_t', p_t)

  ## prediction model
  coefficient = [p_x_t1, p_y_t1, v_x_t1, v_y_t1, a_x_t1, a_y_t1, yaw_t1, yaw_rate_t1]
  F = Matrix([
    [p_t],
    [v_t],
    [a_t],
    [yaw_t],
    [yaw_rate_t]
  ])

  latex_export_string += export('\\textbf{F}', F)

  jacobian = Matrix([
    [p_t],
    [v_t],
    [a_t],
    [yaw_t],
    [yaw_rate_t]
  ]).jacobian(coefficient)

  latex_export_string += export('\\textbf{J}_t', jacobian)

  return latex_export_string
  
if __name__ == "__main__":
  init_printing()
  print('latex output:\n\n\n' + do_formula())
