function[xr,ur]=optimalss(LTI,dim,weight,constraints,eqconstraints)

H=blkdiag(zeros(dim.nx),eye(dim.nu));
h=zeros(dim.nx+dim.nu,1);


options1 = optimoptions(@quadprog); 
options1.OptimalityTolerance=5e-2;
options1.ConstraintTolerance=5e-2;
options1.MaxIterations = 1e3;
% options1.Display='off';
xur=quadprog(H,h,[],[],eqconstraints.A,eqconstraints.b,[],[],[],options1);
xr=xur(1:dim.nx);
ur=xur(dim.nx+1:end);

end