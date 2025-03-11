function [H,h]=costgen(predmod,weight,dim)

Qbar=blkdiag(kron(eye(dim.N),weight.Q),weight.P);
H=predmod.S'*Qbar*predmod.S+kron(eye(dim.N),weight.R);   
h=predmod.S'*Qbar*predmod.T;