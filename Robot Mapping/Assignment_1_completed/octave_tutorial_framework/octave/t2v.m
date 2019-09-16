%computes the pose vector v from an homogeneous transform A
function p=t2v(M)
	p(1,1) = M(1,3);
    p(2,1) = M(2,3);
	p(3,1)=atan2(M(2,1),M(1,1));
end
