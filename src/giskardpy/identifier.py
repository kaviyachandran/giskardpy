world = [u'world']
robot = world + [u'robot']
fk_pose = robot + [u'get_fk_pose']
fk_np = robot + [u'get_fk_np']
joint_states = robot + [u'joint_state']

constraints_identifier = [u'constraints']
trajectory = [u'traj']
time = [u'time']
cmd = [u'cmd']
last_cmd = [u'last_cmd']
closest_point = [u'cpi']
collisions = [u'collisions']
collision_goal_identifier = [u'collision_goal']
soft_constraint_identifier = [u'soft_constraints']
execute = [u'execute']
next_move_goal = [u'next_move_goal']
qp_data = [u'qp_data']
A = qp_data + [u'A']
H = qp_data + [u'H']
lbA = qp_data + [u'lbA']
ubA = qp_data + [u'ubA']
lb = qp_data + [u'lb']
ub = qp_data + [u'ub']
xdot_full = qp_data + [u'xdot_full']
weight_keys = qp_data + [u'weight_keys']
b_keys = qp_data + [u'b_keys']
bA_keys = qp_data + [u'bA_keys']
xdot_keys = qp_data + [u'xdot_keys']




#stuff from rosparam
rosparam = [u'rosparam']

#symengine stuff
symbolic_backend = rosparam + [u'symbolic_backend']
symengine_backend = rosparam + [u'backend']
symengine_opt_level = rosparam + [u'opt_level']

data_folder = rosparam + [u'path_to_data_folder']
gui = rosparam + [u'enable_gui']
map_frame = rosparam + [u'map_frame']
robot_description = [u'robot_description']
marker_visualization = rosparam + [u'enable_visualization']
sample_period = rosparam + [u'sample_period']

fft_duration = rosparam + [u'fft_duration']
wiggle_detection_threshold = rosparam + [u'wiggle_detection_threshold']
min_wiggle_frequency = rosparam + [u'min_wiggle_frequency']

zero_weight_distance = rosparam + [u'zero_weight_distance']
low_weight_distance = rosparam + [u'low_weight_distance']
max_weight_distance = rosparam + [u'max_weight_distance']
collisions_distances = rosparam + [u'collision_avoidance']
default_collision_distances = collisions_distances + [u'default']

joint_weights = rosparam + [u'joint_weights']
default_joint_weight_identifier = joint_weights + [u'default']

joint_vel = rosparam + [u'joint_vel_limit']
default_joint_vel = joint_vel + [u'default']

joint_acc = rosparam + [u'joint_acceleration_limit']
default_joint_acc = joint_acc + [u'default']

nWSR = rosparam + [u'nWSR']
joint_convergence_threshold = rosparam + [u'joint_convergence_threshold']
collision_time_threshold = rosparam + [u'collision_time_threshold']
fill_velocity_values = rosparam + [u'fill_velocity_values']
debug = rosparam + [u'debug']
plot_trajectory = rosparam + [u'plot_trajectory']
enable_collision_marker = rosparam + [u'enable_collision_marker']
ignored_self_collisions = rosparam + [u'self_collision_matrix', u'ignore']
added_self_collisions = rosparam + [u'self_collision_matrix', u'add']
