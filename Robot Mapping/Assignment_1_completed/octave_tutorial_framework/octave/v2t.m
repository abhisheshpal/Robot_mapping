%computes the homogeneous transform matrix A of the pose vector v
function M=v2t(p)
	M=[cos(p(3)), -sin(p(3)), p(1);
	sin(p(3)),  cos(p(3)), p(2);
	0   0  1  ];
end
