foo = -1; # useless variable, this is just to prevent octave to go crazy


function plot_XR(XR_estimate, XR_initial, XR_true)
	num_poses = size(XR_estimate, 3);

	subplot (2,2,1, "align");
	for i=1:num_poses
		tmp = XR_estimate(:,:,i);
		tmp = t2v_global_to_local(tmp);
		plot(tmp(1), tmp(2),'g*',"linewidth",2);
		hold on;
	endfor
	title("Estimated poses");
	axis([-15, 20, -15, 15]);

	subplot (2,2,2, "align");
	for i=1:num_poses
		tmp = XR_initial(:,:,i);
		tmp = t2v_global_to_local(tmp);
		plot(tmp(1), tmp(2),'g*',"linewidth",2);
		hold on;
	endfor
	title("Initial poses");
	axis([-15, 20, -15, 15]);

	subplot (2,2,3, "align");
	for i=1:num_poses
		tmp = XR_true(:,:,i);
		tmp = t2v_global_to_local(tmp);
		h = plot(tmp(1), tmp(2),'g*',"linewidth",2);
		hold on;
	endfor
	title("True poses");
	axis([-15, 20, -15, 15]);

	waitfor(h);
endfunction


function plot_XL(XL_estimate, XL_initial, XL_true)
	num_landmarks = size(XL_estimate, 2);

	subplot (2,2,1, "align");
	plot(XL_estimate(1,:), XL_estimate(2,:),'b*',"linewidth",2);
	title("Estimated landmarks");
	axis([-15, 20, -15, 15]);

	subplot (2,2,2, "align");
	plot(XL_initial(1,:), XL_initial(2,:),'b*',"linewidth",2);
	title("Initial landmarks");
	axis([-15, 20, -15, 15]);

	subplot (2,2,3, "align");
	h = plot(XL_true(1,:), XL_true(2,:),'b*',"linewidth",2);
	title("True landmarks");
	axis([-15, 20, -15, 15]);

	waitfor(h);
endfunction


function T = plot_XR_in_one_figure(XR_estimate, XR_initial, XR_true)
	num_poses = size(XR_estimate, 3);
	euclidean_XR = zeros(2, num_poses);
	figure(1);

	for i=1:num_poses
		tmp = XR_estimate(:,:,i);
		tmp = t2v_global_to_local(tmp);
		euclidean_XR(1:2, i) = tmp(1:2, 1);
	endfor
	plot(euclidean_XR(1,:), euclidean_XR(2,:), 'g*', "linewidth", 2);
	hold on;
	axis([-15, 20, -15, 15]);

	for i=1:num_poses
		tmp = XR_initial(:,:,i);
		tmp = t2v_global_to_local(tmp);
		euclidean_XR(1:2, i) = tmp(1:2, 1);
	endfor
	plot(euclidean_XR(1,:), euclidean_XR(2,:), 'b*', "linewidth", 2);
	hold on;
	axis([-15, 20, -15, 15]);

	for i=1:num_poses
		tmp = XR_true(:,:,i);
		tmp = t2v_global_to_local(tmp);
		euclidean_XR(1:2, i) = tmp(1:2, 1);
	endfor
	h = plot(euclidean_XR(1,:), euclidean_XR(2,:), 'ro', "linewidth", 2);
	hold on;
	axis([-15, 20, -15, 15]);

	waitfor(h);
endfunction


function plot_XL_in_one_figure(XL_estimate, XL_initial, XL_true)
	figure(2);
	plot(XL_estimate(1,:), XL_estimate(2,:), 'g*', "linewidth", 2);
	hold on;
	plot(XL_initial(1,:), XL_initial(2,:), 'b*', "linewidth", 2);
	hold on;
	h = plot(XL_true(1,:), XL_true(2,:), 'ro', "linewidth", 2);
	hold on;
	legend("estimate","initial guess","ground truth");
	axis([-15, 20, -15, 15]);
	waitfor(h);
endfunction
