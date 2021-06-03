function J = J_evaTarget(x_eva,x_pur,u_eva,u_pur,C)

[~,k]=size(x_eva);
k=k-1;  %x_pur has size (nX, planningThreshold+1)
J=0;
for ik=1:k
    e=(x_eva(:,ik+1)-x_pur(:,ik+1)); %look for NEXT location
    eT=x_eva(1:2,ik+1)-C.x_target;
    J = J + -e'*C.Q*e + u_eva(:,ik)'*C.Rself*u_eva(:,ik) + ...
        u_pur(:,ik)'*C.Ropp*u_pur(:,ik) + ...
        eT'*C.Q_target*eT;
end

end
