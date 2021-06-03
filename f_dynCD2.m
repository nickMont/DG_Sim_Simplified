function xkp1 = f_dynCD2(x0,u,dt,vk,params)
n=length(x0)/2;
xkp1 = [eye(n) dt*eye(n); zeros(n,n) (1-params.cd)*eye(n)]*x0+...
    [zeros(n,1);params.cd*-x0(n+1:end)*norm(x0(n+1:end))]+...
    [eye(n)*dt^2/2;eye(n)*dt]*u+...
    [eye(n)*dt^2/2;eye(n)*dt]*vk;

% cd=0.8;
% A=[zeros(n,n) eye(n); zeros(n,n) -cd*eye(n)];
% F=expm(A*dt);

end

