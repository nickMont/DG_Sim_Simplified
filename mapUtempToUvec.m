function uVec = mapUtempToUvec(utemp, uType, uTypeDat)
% utemp is default data vec on -1-to-1
% uType is the type of data (-2 to 2 in all directions, circular control)
%   Circle: utemp(1,x) is the radius, utemp(2,x) is the angle fraction
% uTypeDat is the extra data necessary

if strcmp(uType,"circle")
    rmax=uTypeDat.radius;
    [nU,n]=size(utemp);
    uVec=zeros(nU,1); %ignore duplicate 0 radius
    %choose utemp(1,i)=radius, utemp(2,i)=theta.  Ignore negative radius.
    cc=1;
    for ij=1:n
        theta=utemp(2,ij)*pi;
        if(utemp(1,ij)>0 && theta>=(-pi+0.001)) %don't duplicate 0 radius or pi=-pi
            cc=cc+1; %first iteration has cc=2, forces u=[0;0] to start
            r=rmax*utemp(1,ij);
            uV1=r*cos(theta);
            uV2=r*sin(theta);
            uVec=[uVec [uV1; uV2]];
        end
    end
end

end

