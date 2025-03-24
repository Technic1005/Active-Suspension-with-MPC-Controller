function eqconstraints=eqconstraintsgen(LTI,dim,dtilde)

eqconstraints.A=[eye(dim.nx)-LTI.A -LTI.B; LTI.C, LTI.D;
    0,0,0,0,0,0,0,0,0,0,1,0,0;
    0,0,0,0,0,0,0,0,0,0,0,1,0;
    0,0,0,0,0,0,0,0,0,0,0,0,1;
    1,0,0,0,0,0,0,0,0,0,0,0,0;
    0,1,0,0,0,0,0,0,0,0,0,0,0;
    0,0,1,0,0,0,0,0,0,0,0,0,0;
    0,0,0,1,0,0,0,0,0,0,0,0,0;
    ];
eqconstraints.b=[LTI.Bd*dtilde; LTI.yref-LTI.Cd*dtilde;0;0;0;0;0;0;0
    ];

end
