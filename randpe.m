function v = randpe(nx,ny,pOf0,logmin,logmax)
v=zeros(nx,ny);
for ij=1:nx
    for ik=1:ny
        isZero=rand;
        if(isZero<=pOf0)
            v(ij,ik)=0;
        else
            v(ij,ik)=10^(rand*(logmax-logmin)+logmin);
        end
    end
end
end

