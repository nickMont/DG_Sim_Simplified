clear;clc;

%rngseedno=10;
rngseedno=40;
rng(rngseedno)

plotFlag=0;
plotEndFlag=1;

tstep=1;
tmax=20;

cdPur=0;
cdEva=0;

%utemp=permn(-2:.5:2,2)';
%utemp=permn(-2:0.5:2,2)';
utemp=permn(-1:.25:1,2)';
upmax=2;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);
max_steps_to_predict=1;
utemp_perm = permuteOverTime(utemp,max_steps_to_predict);

Game.uType="velstep";
%Options: velstep (step of constant size, from velocity)
%         accel (double integrator)
Game.dt=tstep;

% Red and blue teams, Sred and Sblue
% S contains state information

Qpur = diag([10 10 0 0]); Rpur = diag([0.1 1]);
Qeva = diag([10 10 0 0]); Reva = diag([5 10]);

qrTrue=[diag(Qeva);diag(Reva)];

eye2=eye(2); zer2=zeros(2,2);
Hpur=[eye2 zer2 zer2 zer2;
    zer2 zer2 eye2 zer2];
Heva=Hpur;
Rnoisepur=0.2*eye(4);
Rnoiseeva=Rnoisepur;
H14=[Hpur zeros(4,6)];

x_target = [0;0];
Q_target_pur = 5*eye(2);
Q_target_eva = 5*eye(2);

% Ppur=Peva;
%This can be shunted off to a separate function
n=0; %n is now the iteration index (yes, misuse of var names)
xTrue=[[0 0 1.0 1.0]'; [10 4 0 0]'];
xPur=xTrue;
xEva=xTrue;
axisveck=[-20 20 -5 35];

if plotFlag==1
figure(1);clf
f1=scatter(xTrue(1),xTrue(2),'b');
hold on
f2=scatter(xTrue(5),xTrue(6),'r');
axis(axisveck)
end

% wStore{1}=wloc;
% xPartStore{1}=xPur_part;
xPurS{1}=xPur;
xEvaS{1}=xEva;
xTrueS{1}=xTrue;
dJS=[];
Jp0=0;

for ij=1:tstep:tmax
    n=n+1
    tic
    
    xPurMean=[xPur;diag(Qeva);diag(Reva)];
    
    %Pursuer and evader controller are run "separately" with their own sets
    % of measurements.  I've stripped out the measurement/filtering code
    % here since we have code onboard the quads to do that.
    
    %Preload GT params used in one or more GT solvers
    gameState_p.xPur=xPurMean(1:4);
    gameState_p.xEva=xPurMean(5:8);
    gameState_p.dt=tstep;
    gameState_p.kMax=max_steps_to_predict;
    gameState_p.nu=2;
    Spur_p.uMat={0}; Seva_p.uMat={0};
    for ik=1:max(size(utemp_perm))
        Spur_p.uMat{ik}=squeeze(utemp_perm(:,:,ik));
    end
    Spur_p.Jname='J_purTarget';
    Spur_p.fname='f_dynPur';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(2,2);
    Spur_p.Jparams.x_target = x_target;
    Spur_p.Jparams.Q_target = Q_target_pur;
    Spur_p.params.cd=cdPur;
    Seva_p.uMat = Spur_p.uMat;
    Seva_p.Jname='J_evaTarget';
    Seva_p.fname='f_dynEva';
    Seva_p.Jparams.Q=Qeva;
    Seva_p.Jparams.Rself=Reva;
    Seva_p.Jparams.Ropp=zeros(2,2);
    Seva_p.Jparams.x_target = x_target;
    Seva_p.Jparams.Q_target = Q_target_eva;
    Seva_p.params.cd=cdEva;
    [uPurTrue,uEvaEst,flag]=f_dyn2(Spur_p,Seva_p,gameState_p,zeros(4,1));
    
    %More params, most of this can be loaded outside of the loop but the
    %cost is low and it's easier to put it here in case you want to change
    %something mid-loop.
    gameState_e.xPur=xEvaMean(1:4);
    gameState_e.xEva=xEvaMean(5:8);
    gameState_e.dt=tstep;
    gameState_e.kMax=max_steps_to_predict;
    gameState_e.nu=2;
    Spur_e.uMat={0}; Seva_e.uMat={0};
    for ik=1:max(size(utemp_perm))
        Spur_e.uMat{ik}=squeeze(utemp_perm(:,:,ik));
    end
    Spur_e.Jname='J_purTarget';
    Spur_e.fname='f_dynPur';
    Spur_e.Jparams.Q=Qpur;
    Spur_e.Jparams.Rself=Rpur;
    Spur_e.Jparams.Ropp=zeros(2,2);
    Spur_e.Jparams.x_target = x_target;
    Spur_e.Jparams.Q_target = Q_target_pur;
    Spur_e.params.cd=cdPur;
    Seva_e.uMat = Spur_e.uMat;
    Seva_e.Jname='J_evaTarget';
    Seva_e.fname='f_dynEva';
    Seva_e.Jparams.Q=Qeva;
    Seva_e.Jparams.Rself=Reva;
    Seva_e.Jparams.Ropp=zeros(2,2);
    Seva_e.Jparams.x_target = x_target;
    Seva_e.Jparams.Q_target = Q_target_eva;
    Seva_e.params.cd=cdEva;
    gameState_e = gameState_p;
    gameState_e.xPur=xEva(1:4);
    gameState_e.xEva=xEva(5:8);
    [uPurEst,uEvaTrue,flag]=f_dyn2(Spur_e,Seva_e,gameState_e,zeros(4,1));
    
    %actual propagation
    xTrue(1:4)=f_dynPur(xTrue(1:4),uPurTrue(:,1),tstep,zeros(2,1),Spur_p.params);
    xTrue(5:8)=f_dynEva(xTrue(5:8),uEvaTrue(:,1),tstep,zeros(2,1),Seva_e.params);
    
    %estimated stats (note: we get these from quad sources so I've removed
    %the estimators)
    xPurMean=xTrue;
    xEvaMean=xTrue;
    
    %storage
    xTrueS{n+1}=xTrue;
    
    %cost calculation, need to debug error index length in J_pur
    e=xTrue(1:4)-xTrue(5:8);
    Jloc = e'*Spur_p.Jparams.Q*e + uPurTrue(:,1)'*Spur_p.Jparams.Rself*uPurTrue(:,1) + ...
        uEvaTrue(:,1)'*Spur_p.Jparams.Ropp*uEvaTrue(:,1);
    Jp0=Jp0+Jloc;
    
    if plotFlag==1
    figure(1)
    pause(.1)
    delete(f1); delete(f2)
    f1=scatter(xTrue(1),xTrue(2),'b');
    hold on
    f2=scatter(xTrue(5),xTrue(6),'r');
    axis(axisveck)
    end
    
    tThisStep=toc
end

if plotEndFlag==1
%     figure(2);clf;
%     subplot(3,1,1);
%     plot(1:n+1,dJS(1,:),'-.r');
%     hold on
%     plot(1:n+1,dJS(2,:),'-ob');
%     legend('\DeltaQ_{xx}','\DeltaQ_{yy}')
%     xlabel('Time (s)');
%     ylabel('Cost parameter (m^{-2})');
%     figset
%     
%     subplot(3,1,2);
%     plot(1:n+1,dJS(3,:),'-.r');
%     hold on
%     plot(1:n+1,dJS(4,:),'-ob');
%     legend('\DeltaQ_{vx}','\DeltaQ_{vy}');
%     xlabel('Time (s)');
%     ylabel('Cost parameter (m^{-2}s^2)');
%     figset
%     
%     subplot(3,1,3);
%     plot(1:n+1,dJS(5,:),'-.r');
%     hold on
%     plot(1:n+1,dJS(6,:),'-ob');
%     legend('\DeltaR_{x}','\DeltaR_{y}');
%     xlabel('Time (s)');
%     ylabel('Cost parameter (m^{-2}s^4)');
%     figset
    
    xP=zeros(2,n+1);xE=zeros(2,n+1);
    for ijk=1:n+1
        xP(:,ijk)=xTrueS{ijk}(1:2); xE(:,ijk)=xTrueS{ijk}(5:6);
    end
    figure(3);clf;
    plot(xP(1,:),xP(2,:),'-xr');
    hold on
    plot(xE(1,:),xE(2,:),'-ob');
    title('Interceptor using velocity matching vs unaware evader');
    xlabel('East displacement (m)');
    ylabel('North displacement (m)');
    legend('Pursuer','Evader');
    axis([0 90 0 10])
    figset
    
end
