function output = permuteOverTime(vec,tstep)
[rows,num]=size(vec);
output=zeros(rows,tstep,num^tstep);
n=0;
%too tired to do this right
if tstep==1
    for ij=1:num
        n=n+1;
        output(:,1,n)=vec(:,ij);
    end
elseif tstep==2
   for ij=1:num
        for ik=1:num
            n=n+1;
            output(:,1,n)=vec(:,ij);
            output(:,2,n)=vec(:,ik);
        end
   end
elseif tstep==3
   for ij=1:num
        for ik=1:num
            for iL=1:num
                n=n+1;
                output(:,1,n)=vec(:,ij);
                output(:,2,n)=vec(:,ik);
                output(:,3,n)=vec(:,iL);
            end
        end
   end
elseif tstep==4
    for ij=1:num
        for ik=1:num
            for iL=1:num
                for im=1:num
                    n=n+1;
                    output(:,1,n)=vec(:,ij);
                    output(:,2,n)=vec(:,ik);
                    output(:,3,n)=vec(:,iL);
                    output(:,4,n)=vec(:,im);
                end
            end
        end
    end
elseif tstep==5
    for ij=1:num
        for ik=1:num
            for iL=1:num
                for im=1:num
                    for iN=1:num
                        n=n+1;
                        output(:,1,n)=vec(:,ij);
                        output(:,2,n)=vec(:,ik);
                        output(:,3,n)=vec(:,iL);
                        output(:,4,n)=vec(:,im);
                        output(:,5,n)=vec(:,iN);
                    end
                end
            end
        end
    end
else
    output=[];
end

end

