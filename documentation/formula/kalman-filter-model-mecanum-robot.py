import sage.all
from sage.all import matrix, vector, var, latex, expand, cos, sin, diff, function

def export(left_side : str, right_side_sage_element) -> str:
  print(left_side + ' = \n' + str(right_side_sage_element))
  return left_side + ' = \n' + latex(right_side_sage_element) + '\n'

def create_kalman_filter_model_mecanum() -> str:
  latex_export_string: str = str()

  # prediction moving equations
  var('dt')
  ## acceleration of robot is constant
  ## in robot coordinate system
  var('a_x a_y')
  var('a_x_t1 a_y_t1')
  a_t = matrix([[a_x], [a_y]])
  latex_export_string += export('a_t', a_t)
  a_t1 = matrix([[a_x_t1], [a_y_t1]])
  
  a_t = a_t1

  latex_export_string += export('a_{t-1}', a_t1)
  latex_export_string += export('a_t', a_t)

  ## velocity
  ## in robot coordinate system
  var('v_x v_y')
  var('v_x_t1 v_y_t1')
  v = matrix([[v_x], [v_y]])
  v_t1 = matrix([[v_x_t1], [v_y_t1]])
  latex_export_string += export('v_t', v)

  v = v_t1 + a_t1 * dt

  latex_export_string += export('v_{t-1}', v_t1)
  latex_export_string += export('v_t', v)

  ## yaw
  ## in world coordinate system
  var('yaw yaw_t1')
  var('yaw_dot yaw_dot_t1')
  yaw_dot = yaw_dot_t1
  yaw = yaw_t1 + yaw_dot_t1 * dt
  latex_export_string += export('yaw_t \\frac{d}{dt}', yaw_dot)
  latex_export_string += export('yaw_t', yaw)

  ## position
  ## in world coordinate system
  var('R_t1')
  var('p p_t1')
  var('p_x_t1 p_y_t1')
  p_t1 = matrix([[p_x_t1], [p_y_t1]])
  R_t1 = matrix([
    [ cos(yaw_t1), -sin(yaw_t1) ],
    [ sin(yaw_t1),  cos(yaw_t1) ]
  ])
  latex_export_string += export('R_{t-1}', R_t1)
  p = p_t1  + R_t1 * v_t1 * dt + R_t1 * 1/2 * matrix([[a_x_t1**2], [a_y_t1**2]]) * dt
  latex_export_string += export('p_t', p)

  ## prediction model
  # jacobian = vector(p[0,:])
  # f = function(p[0,:])

  return latex_export_string

if __name__ == "__main__":
  print('latex output:\n\n\n' + create_kalman_filter_model_mecanum())
