from sympy import Matrix, Symbol, MatrixSymbol, cos, sin, init_printing, latex, simplify

def export(left_side : str, right_side_sage_element) -> str:
  print(left_side + ' = \n' + str(right_side_sage_element))
  return left_side + ' = \n' + latex(right_side_sage_element) + '\n\n'

def do_prediction_model() -> str:
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
  yaw_rate_t1 = Symbol('\\frac{d}{dt}\\phi_{z_{t-1}}')
  yaw_rate_t = yaw_rate_t1
  yaw_t = yaw_t1 + yaw_rate_t1 * dt

  latex_export_string += export('\\phi_{z_t}', yaw_t)
  latex_export_string += export('\\frac{d}{dt}\\phi_{z_t}', yaw_rate_t)

  ## position
  ## in world coordinate system
  R_t1 = Matrix([
    [cos(yaw_t1), -sin(yaw_t1)],
    [sin(yaw_t1),  cos(yaw_t1)]
  ])
  p_x_t1 = Symbol('p_x_(t-1)')
  p_y_t1 = Symbol('p_y_(t-1)')
  p_t1 = Matrix([[p_x_t1], [p_y_t1]])
  
  p_t = p_t1 + R_t1 * v_t1 * dt + 1.0/2.0 * R_t1 * a_t1 * dt**2

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
  

def do_system_noise_model() -> str:
  latex_export_string: str = str()

  # system noise equations
  dt = Symbol('dt')
  yaw_t1 = Symbol('\\phi_{z_{t-1}}')
  jerk_noise_variance = Symbol('\\sigma^2_\\textbf{j}')
  R = Matrix([
    [cos(yaw_t1), -sin(yaw_t1)],
    [sin(yaw_t1),  cos(yaw_t1)]
  ])

  ## jerk noise part
  ### jerk
  j_x = Symbol('j_x')
  j_y = Symbol('j_y')
  j = Matrix([[j_x], [j_y]])
  
  latex_export_string += export('\\textbf{j}', j)

  ### acceleration
  a = j * dt

  latex_export_string += export('\\textbf{a}', a)

  ### velocity
  v = a * dt

  latex_export_string += export('\\textbf{v}', v)

  ### position
  p = R * v * dt

  latex_export_string += export('\\textbf{p}', p)

  ### building vector
  jerk_noise = Matrix([
    [p[0, 0]],
    [p[1, 0]],
    [v[0, 0]],
    [v[1, 0]],
    [a[0, 0]],
    [a[1, 0]],
    [0], # yaw
    [0]  # yaw rate
  ])
  Q_jerk = jerk_noise_variance * jerk_noise * jerk_noise.transpose()

  latex_export_string += export('\\textbf{j}_{\\textrm{noise}}', jerk_noise)
  latex_export_string += export('\\textbf{Q}_{\\textbf{j}}', Q_jerk)


  ## yaw acceleration part
  ### yaw acceleration
  yaw_acceleration = Symbol('\\frac{d^2}{dt^2}\\phi_z')
  yaw_acceleration_variance = Symbol('\\sigma^2_{\\textrm{yaw}}')

  ### yaw rate
  yaw_rate = yaw_acceleration * dt

  latex_export_string += export('\\frac{d}{dt}\\phi_z', yaw_rate)

  ### yaw
  yaw = yaw_rate * dt

  latex_export_string += export('\\phi_z', yaw)

  ### building vector
  yaw_noise = Matrix([
    [0], # pos x
    [0], # pos y
    [0], # vel x
    [0], # vel y
    [0], # acc x
    [0], # acc y
    [yaw],
    [yaw_rate]
  ])
  Q_yaw = yaw_acceleration_variance * yaw_noise * yaw_noise.transpose()

  latex_export_string += export('\\frac{d^2}{dt^2}\\phi_{z_\\textrm{noise}}', yaw_noise)
  latex_export_string += export('\\textbf{Q}_{\\textrm{yaw}}', Q_yaw)


  ## final system noise matrix
  Q = Q_jerk + Q_yaw

  latex_export_string += export('\\textbf{Q}', Q)

  return latex_export_string

if __name__ == "__main__":
  init_printing()
  print('latex output prediction model:\n\n' + do_prediction_model())
  print('latex output system noise model\n\n' + do_system_noise_model())
