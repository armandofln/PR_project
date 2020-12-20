close all
clear
clc
graphics_toolkit("fltk")

addpath '../tools/g2o_wrapper'

source "geometry_helpers_global_to_local.m"
source "visualization.m"
source "odometry.m"
source "triangulation.m"
source "LS_optimization.m"



##########################################
# DATASET

[_, _, transitions, observations] = loadG2o('../datasets/slam2D_bearing_only_initial_guess.g2o');
[true_landmarks, true_poses, _, _] = loadG2o('../datasets/slam2D_bearing_only_ground_truth.g2o');

# initial poses: odometry
initial_poses = compute_initial_odometry(true_poses, transitions);

# initial landmarks: triangulation
[initial_landmarks, valid_id_to_state_map] = triangulate_landmarks(initial_poses, observations);

disp(""); # leave a blank line



##########################################
# XR and XL (initial and true)

# dimensions
num_poses = length(initial_poses);
num_landmarks = length(initial_landmarks);

# XR
XR_initial = zeros(3, 3, num_poses);
XR_true = zeros(3, 3, num_poses);

for i=1:num_poses
	XR_initial(:,:,i) = v2t_global_to_local([initial_poses(i).x, initial_poses(i).y, initial_poses(i).theta]);
	XR_true(:,:,i) = v2t_global_to_local([true_poses(i).x, true_poses(i).y, true_poses(i).theta]);
endfor

# XL
XL_initial = zeros(2, num_landmarks);
XL_true = zeros(2, num_landmarks);

for i=1:num_landmarks
	Xl = initial_landmarks(i);
	XL_initial(:,i) = [Xl.x_pose; Xl.y_pose];
	landmark_id = Xl.id;
	for j=1:length(true_landmarks)
		# initial_landmarks are not in the same order as the true_landmarks,
		# so we need to look for them
		if (true_landmarks(j).id == landmark_id)
			Xl = true_landmarks(j);
			XL_true(:,i) = [Xl.x_pose; Xl.y_pose];
		endif
	endfor
endfor



##########################################
# LEAST SQUARES

damping = 0.05;
num_iterations = 200;
kernel_threshold = 1;

[XR_estimate, XL_estimate] = least_squares(XR_initial,
										   XL_initial,
										   observations,
										   valid_id_to_state_map, # this is where i store my associations
										   num_poses,
										   num_landmarks,
										   num_iterations,
										   damping,
										   kernel_threshold);



##########################################
# PRINT

disp("\nPlotting the poses (it will take just a moment)...");
plot_XR(XR_estimate, XR_initial, XR_true);

disp("\nPlotting the landmarks...");
plot_XL(XL_estimate, XL_initial, XL_true);
