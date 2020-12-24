source "../tools/utilities/geometry_helpers_2d.m" # for J_atan2()
source "geometry_helpers_global_to_local.m"


function [e,Jr,Jl]=errorAndJacobian(Xr, Xl, z)
	R = Xr(1:2,1:2);
	t = Xr(1:2,3);
	vec_z_hat = R*Xl + t;
	J_atan = J_atan2(vec_z_hat);
	c = R(1, 1);
	s = R(1, 2);

	# error
	z_hat = atan2(vec_z_hat(2), vec_z_hat(1));
	e = z_hat - z;

	# Jr
	#
	# Jr is different from the usual because I'm using v2t_global_to_local()
	# ( the inverse of v2t() ) in h(x), so the jacobian is different
	Jr_vec_z_hat = zeros(2,3);
	Jr_vec_z_hat(1:2,1:2) = -eye(2);
	Jr = J_atan * Jr_vec_z_hat;
	Jr(3) = -1;

	# Jl
	Jl_vec_z_hat = R;
	Jl = J_atan * Jl_vec_z_hat;
endfunction


function [XR, XL]=boxPlus(XR, XL, num_poses, num_landmarks, dx)
	index_ = 1;
	for(pose_index=1:num_poses)
		dxr=dx(index_:index_+2);
		XR(:,:,pose_index)=v2t_global_to_local(dxr)*XR(:,:,pose_index);
		index_ += 3;
	endfor

	index_ = 3*num_poses + 1;
	for(landmark_index=1:num_landmarks)
		dxl=dx(index_:index_+1,:);
		XL(:,landmark_index)+=dxl;
		index_ += 2;
	endfor
endfunction


function [XR_estimate, XL_estimate] = least_squares(XR_initial,
													XL_initial,
													observations,
													associations,
													transitions,
													num_poses,
													num_landmarks,
													num_iterations,
													damping,
													kernel_threshold)

	system_size = 3*num_poses + 2*num_landmarks;

	XR_estimate = XR_initial;
	XL_estimate = XL_initial;

	for iteration=1:num_iterations

		printf('Iteration %d of %d\n', iteration, num_iterations);

		H=zeros(system_size, system_size);
		b=zeros(system_size,1);

		for pose_index=2:num_poses
			observations_set_index = pose_index - 1; # the n-th pose generates the (n-1)-th set of observations
			for observation_index=1:length(observations(observations_set_index).observation)
				current_observation = observations(observations_set_index).observation(observation_index);
				landmark_index = associations(current_observation.id + 1);
				if (landmark_index!=0)
					z = current_observation.bearing;
					Xr = XR_estimate(:,:,pose_index);
					Xl = XL_estimate(:,landmark_index);

					[e,Jr,Jl] = errorAndJacobian(Xr, Xl, z);

					chi=e'*e;
					if (chi>kernel_threshold)
						continue;
					endif

					Hrr=Jr'*Jr;
					Hrl=Jr'*Jl;
					Hll=Jl'*Jl;
					br=Jr'*e;
					bl=Jl'*e;

					Xr_index = 3*(pose_index-1) + 1;
					Xl_index = 3*num_poses + 1;
					Xl_index += 2*(landmark_index-1);

					H(Xr_index:Xr_index+2, Xr_index:Xr_index+2) += Hrr;
					H(Xr_index:Xr_index+2, Xl_index:Xl_index+1) += Hrl;
					H(Xl_index:Xl_index+1, Xl_index:Xl_index+1) += Hll;
					H(Xl_index:Xl_index+1, Xr_index:Xr_index+2) += Hrl';

					b(Xr_index:Xr_index+2) += br;
					b(Xl_index:Xl_index+1) += bl;
				endif
			endfor
		endfor

		H += eye(system_size) * damping;
		dx = zeros(system_size,1);
		dx(4:end) =-(H(4:end,4:end)\b(4:end,1));

		[XR_estimate, XL_estimate] = boxPlus(XR_estimate,
											 XL_estimate,
											 num_poses,
											 num_landmarks,
											 dx);
	endfor

	# to include the first transition into account:
	T = v2t_global_to_local(transitions(1).v);
	T = T * XR_estimate(:,:,1);
	T = T * inv(XR_estimate(:,:,2));
	for pose_index=2:num_poses
		Xr = XR_estimate(:,:,pose_index);
		Xr = Xr * T;
		XR_estimate(:,:,pose_index) = Xr;
	endfor
	T = inv(T);
	for landmark_index=1:num_landmarks
		tmp = [XL_estimate(:,landmark_index); 1];
		tmp = T * tmp;
		XL_estimate(:,landmark_index) = tmp(1:2);
	endfor

endfunction
