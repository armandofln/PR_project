foo = -1; # useless variable, this is just to prevent octave to go crazy

# For the triangulation I just used the intersection between two lines.
# The first line is computed from the first observation encountered.
# The second line is computed from the first VALID observation encountered after
# the first observation. To be valid, the second observation needs to be far enough
# and with an absolute angle different enough from the first one


# for ONE landmark
function out = triangulate_landmark(pose_1, observation_1, pose_2, observation_2)
	angle_1 = pose_1.theta + observation_1.bearing; # absolute angle
	angle_2 = pose_2.theta + observation_2.bearing; # absolute angle

	angle_check = abs(angle_1-angle_2);
	angle_check = rem(angle_check, (2*pi)); # angle_check in [0, 2pi]

	distance_check = (pose_1.x - pose_2.x) ^ 2 + (pose_1.y - pose_2.y) ^ 2;
	distance_check = distance_check ^ (.5);

	if ((angle_check<0.3) || (distance_check<0.3))
		out = landmark(-1, [-1,-1]); # a landmark with id == -1 means that the
									 # landmark can't be traingulated from these
									 # two observations
	else
		m_1 = tan(angle_1);
		m_2 = tan(angle_2);
		q_1 = pose_1.y - (pose_1.x * m_1);
		q_2 = pose_2.y - (pose_2.x * m_2);
		x = (q_2 - q_1)/(m_1 - m_2);
		y = (m_1 * x) + q_1;
		out = landmark(observation_1.id, [x,y]);
	endif
end


# for all landmarkS
function [landmarks, id_to_state_map] = triangulate_landmarks(poses, observations)
	id_to_sample_map = zeros(300, 1);
	id_to_state_map = zeros(300, 1);

	for i=2:length(poses) # it starts from 2 because the first pose hasn't any observation
		for j=1:length(observations(i-1).observation)
			id = observations(i-1).observation(j).id; # landmark id

			id += 1; # convert from id landmark to valid id
					 # this is because there's a landmark with id == 0

			sample = id_to_sample_map(id); # now i can use the valid id as an index

			if (sample==0)
				# first time we encounter this landmark, so we just store the observation
				sampled_poses(end+1) = poses(i);
				sampled_observations(end+1) = observations(i-1).observation(j);
				id_to_sample_map(id) = length(sampled_poses);

			elseif (sample!=-1)
				# if sample is neither 0 nor -1 it means that sample is an index of the
				# previous observation of the same landmark. (line 46 of this file)
				tmp = triangulate_landmark(sampled_poses(sample), sampled_observations(sample), poses(i), observations(i-1).observation(j));
				if (tmp.id>-1)
					id_to_sample_map(id) = -1;
					landmarks(end+1) = tmp;
					id_to_state_map(id) = length(landmarks);
				endif
			endif
			# if sample is -1 it means we already triangulated this landmark,
			# so we just skip this observation
		endfor
	endfor
end
