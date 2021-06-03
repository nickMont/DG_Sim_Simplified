function [mm,outcomeRow,outcomeCol]=minimax2(A,B)
%modified from example code by K. Passino

%A is row cost, B is col cost
%[nr,nc]=size(A);
% J1=A;
% J2=B;
[maxvals1,indexmax1]=max(A'); % This is the max value for each row (note transpose)
[minimaxvalue1,secstratR]=min(maxvals1); % Display the security value of P1 and its security strategy
                                         % (P1 loses no more than minimaxvalue1, no matter what strategy 
										 % P2 uses)

% Second, compute the security strategy for player 2 (note difference from computation of security
% strategies for matrix games since both payoff matrices are "loss" matrices, and note that computation of the
% minimax strategies is completely independent)

[maxvals2,indexmax2]=max(B); % This is the min value for each column (note no transpose)
[minimaxvalue2,secstratC]=min(maxvals2); % Display the security value of P2 and its security strategy

% The outcome of the game will be (with a minimax strategy)

outcomeRow=A(secstratR,secstratC);
outcomeCol=B(secstratR,secstratC);
mm=[secstratR;secstratC];

end