import sage.all
from sage.all import matrix, vector, var, latex, expand, cos, sin

def export(left_side : str, right_side_sage_element) -> str:
    print(left_side + ' = \n' + str(right_side_sage_element))
    return left_side + ' = \n' + latex(right_side_sage_element) + '\n'

def create_fleet_to_robot_equation_system() -> str:
    latex_export_string : str = str()

    # Impact of rotation of fleet (omega) to robot velocity.
    var('dx dy')
    vec_d = vector([dx, dy])
    latex_export_string += export('d', vec_d)

    normal_d = vector([-vec_d[1], vec_d[0]]) / vec_d.norm()
    latex_export_string += export('n_d', normal_d)

    var('v_omega_abs')
    v_omega_abs = vec_d.norm()
    latex_export_string += export('|v_omega|', v_omega_abs)
    v_omega = v_omega_abs * normal_d
    latex_export_string += export('v_omega', v_omega)

    var('pi_r')
    R = matrix([[cos(pi_r), -sin(pi_r), 0], [sin(pi_r), cos(pi_r), 0], [0, 0, 1]])
    latex_export_string += export('R', R)

    T_fleet_robot_axis_aligned = matrix([[1, 0, v_omega[0]], [0, 1, v_omega[1]], [0, 0, 1]])
    latex_export_string += export('T_fleet_robot_axis_aligned', T_fleet_robot_axis_aligned)

    T_fleet_robot = R * T_fleet_robot_axis_aligned
    latex_export_string += export('T_fleet_robot', T_fleet_robot)

    return latex_export_string

if __name__ == "__main__":
    print('latex output:\n\n\n' + create_fleet_to_robot_equation_system())
