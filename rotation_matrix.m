syms a b c;

a = -pi/2;
b = pi/4;
c= 0;

Rot_x= [ 1 0 0; 0 cos(a) -sin(a) ; 0 sin(a) cos(a) ];
Rot_y= [ cos(b) 0 sin(b); 0 1 0;-sin(b) 0 cos(b)];
Rot_z = [cos(c) -sin(c) 0 ; sin(c) cos(c) 0 ; 0 0 1];


A = Rot_z*Rot_y*Rot_x;

c= pi/4;
a = pi/2;



Rot_x= [ 1 0 0; 0 cos(a) -sin(a) ; 0 sin(a) cos(a) ];
Rot_y= [ cos(b) 0 sin(b); 0 1 0;-sin(b) 0 cos(b)];
Rot_z = [cos(c) -sin(c) 0 ; sin(c) cos(c) 0 ; 0 0 1];

C = Rot_x*Rot_z;