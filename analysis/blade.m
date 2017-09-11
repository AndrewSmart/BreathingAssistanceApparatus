%From MSc Thesis on 'Turbomachinery Blade Design' by R.C.W. de Koning:
% https://repository.tudelft.nl/islandora/object/uuid%3A9bbcf030-af4b-42c8-a7e5-2157bde13706
%TODO: Can predict minimal gamma (as gamma too low may cause camberline to fall off impeller)
% using a NURBS library to make the nurbs curve using b1_rads & b2_rads, and checking the NURBS curve.
%As gamma moves,x_te,y_te moves.
function te=blade(r1_m,r1a_m,r2_m,gamma,b1_rads)
	%TODO: Iterate up through r1a_m to generate camberlines
	%r1a_m is radius of inlet
	te=[];
	x_le=r1_m;
	y_le=0;
	for i=0:2
		h=i*((r1a_m-r1_m)/2);
		x_le=r1_m+sin(b1_rads)*h;
		y_le=cos(b1_rads)*h;
		m=tan(atan(y_le/x_le)-gamma);
		a=1+m^2;
		b=-2*m*(m*x_le-y_le);
		c=m^2*x_le^2+y_le^2-2*m*x_le*y_le-r2_m^2;
		c_rad=r1_m-r2_m;
		x_te=(-b+sqrt(b^2-4*a*c))/(2*a);
		y_te=y_le+m*(x_te-x_le);
		te=[te;x_te,y_te];
	end
end