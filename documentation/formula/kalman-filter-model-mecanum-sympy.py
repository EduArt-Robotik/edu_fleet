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
  acceleration_noise_variance = Symbol('\\sigma^2_\\textbf{a}')
  R = Matrix([
    [cos(yaw_t1), -sin(yaw_t1)],
    [sin(yaw_t1),  cos(yaw_t1)]
  ])

  ## acceleration noise part
  ## The acceleration can experience a change. Here a equation system is used to calculate the impact on other dimensions.

  ### acceleration
  a_scalar = Symbol('a')
  a = Matrix([[a_scalar], [a_scalar]])
  
  latex_export_string += export('\\textbf{a}', a)

  ### velocity
  v = a * dt

  latex_export_string += export('\\textbf{v}', v)

  ### position
  p = 0.5 * R * a * dt**2

  latex_export_string += export('\\textbf{p}', p)

  ### building vector
  acceleration_noise = Matrix([
    [p[0, 0] / a_scalar],
    [p[1, 0] / a_scalar],
    [v[0, 0] / a_scalar],
    [v[1, 0] / a_scalar],
    [a[0, 0] / a_scalar],
    [a[1, 0] / a_scalar],
    [0], # yaw
    [0]  # yaw rate
  ])
  Q_a = acceleration_noise_variance * acceleration_noise * acceleration_noise.transpose()
  latex_export_string += export('\\textbf{a}_{\\textrm{noise}}', simplify(acceleration_noise))

  ### Q_a
  symbol_a = Symbol('a')
  q_a_helper = Matrix([
    [Symbol('p_x') / a_scalar],
    [Symbol('p_y') / a_scalar],
    [dt],
    [dt],
    [1],
    [1],
    [0],
    [0]
  ])

  Q_a = simplify(acceleration_noise_variance * q_a_helper * q_a_helper.transpose())
  latex_export_string += export('\\textbf{Q}_{\\textbf{a}}', simplify(Q_a / acceleration_noise_variance))


  ## yaw rate part
  yaw_rate_variance = Symbol('\\sigma_{\\dot{\\phi_z}}^2')

  ### yaw rate
  yaw_rate = Symbol('\\dot{\\phi_{z}}')

  latex_export_string += export('\\dot{\\phi_{z}}', yaw_rate)

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
    [yaw / yaw_rate],
    [yaw_rate / yaw_rate]
  ])
  Q_yaw = yaw_rate_variance * yaw_noise * yaw_noise.transpose()

  latex_export_string += export('\\dot{\\phi}_{z_\\textrm{noise}}', yaw_noise)
  latex_export_string += export('\\textbf{Q}_{\\textrm{yaw}}', Q_yaw / yaw_rate_variance)


  ## final system noise matrix
  Q = Q_a + Q_yaw

  latex_export_string += export('\\textbf{Q}', Q)

  return latex_export_string

if __name__ == "__main__":
  init_printing()
  print('latex output prediction model:\n\n' + do_prediction_model())
  print('latex output system noise model\n\n' + do_system_noise_model())
