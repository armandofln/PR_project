foo = -1; # useless variable, this is just to prevent octave to go crazy


function T = v2t_global_to_local(v)
	# this is the inverse of v2t() from geometry_helpers_2d.m
  	c=cos(v(3));
  	s=sin(v(3));
	T = [ c,  s, (-c*v(1))-s*v(2);
         -s,  c, (s*v(1))-c*v(2);
          0,  0,        1       ];
end


function v = t2v_global_to_local(T)
  	c=T(1,1);
  	s=T(1,2);
	theta = atan2(s,c);
	y = -s*T(1,3) -(c*T(2,3));
	x = s*T(2,3) -(c*T(1,3));
	v = [x;y;theta];
end
