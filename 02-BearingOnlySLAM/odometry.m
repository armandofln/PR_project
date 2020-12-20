source "geometry_helpers_global_to_local.m"
addpath '../tools/g2o_wrapper'

# Simple odometry. I use here true_poses(1) as the first pose of initial_poses
# instead of the (0,0,0) pose. The rest of the poses are computed from the
# given transitions.


function initial_poses = compute_initial_odometry(true_poses, transitions)

	first_pose = [true_poses(1).x; true_poses(1).y; true_poses(1).theta];
	tmp(1:3,end+1) = first_pose;

	current_T = v2t_global_to_local(first_pose);

	for i=1:length(transitions)
		current_T = v2t_global_to_local(transitions(i).v) * current_T;
		tmp(1:3,end+1) = t2v_global_to_local(current_T);
	endfor


	for i=1:length(true_poses)
		initial_poses(i) = pose(true_poses(i).id, tmp(1,i), tmp(2,i), tmp(3,i));
	endfor

endfunction
