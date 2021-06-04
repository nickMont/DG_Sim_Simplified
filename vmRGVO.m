function u = vmRGVO(xP,xE,umaxP,umaxE,nX,nU)
% http://cimlab.mie.utoronto.ca/wp-content/uploads/2018/06/Kunwar-2006-Rendezvous-guidance-trajectory-planning-for-robotic-dynamic-obstacle-avoidance-and-interception.pdf
% xP/xE is full state of position/velocity
% returns nan if no control is possible
rP=xP(1:nX);
rE=xE(1:nX);
r=rE-rP;

u=[];

%Find a that satisfies:
% cross(r,rdot)=0; dot(r,rdot)<0;
a=0;
n=1;
cont=true;
nosol=false;
while cont
    a=a+0.1;
    n=n+1;
    rdot=-a*r;
    if(cross(r,rdot)==0 && dot(r,rdot)<0)
        cont=false;
    elseif(n>1000)
        u=nan(nU,1);
        nosol=true;
        cont=false;
    end
end

if ~nosol
    u=rdot;
    if norm(u)>umaxP
        u=u*umaxP/norm(u);
    end
end


end

