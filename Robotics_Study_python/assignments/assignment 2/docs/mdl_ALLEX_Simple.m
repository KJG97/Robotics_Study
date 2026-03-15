% MDL_ALLEX_Simple simplified Kinematic model of LIMS-ALLEX dual-arm robot
%
% MDL_ALLEX_Simple is a script that creates the workspace variables left and
% right which describes the kinematic characteristics of the two 7-joint
% arms of a LIMS3 robot using standard DH conventions.
%
% Also define the workspace vectors:
%   qz    zero joint angle configuration
%   qn    L shaped ready pose
%   qnr   natural ready pose of left arm
%   qnl   natural ready pose of right arm
%
% Notes::
% - SI units of metres are used.
%
% References::
% To be published
%
% See also SerialLink.

% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com


% MODEL: MDL_ALLEX_Simple, IRIM LAB KOREATECH, 7DOF, standard_DH

%Robot parameter
lb  = 0.5;
lp  = 0.195;
lu  = 0.304;
lep = 0.030;
ley = 0.045;
lf  = 0.338;
lw  = 0.015;
lt  = 0.036+0.1;
D2R = (pi/180);

%% Left Arm

%DH parameter
L_left(1) = Link([0 lp  0   -pi/2]); 
L_left(2) = Link([0 0   0   -pi/2]);  L_left(2).offset = -105*D2R;
L_left(3) = Link([0 -lu lep -pi/2]);  L_left(3).offset = -pi/2;
L_left(4) = Link([0 0   ley -pi/2]);  L_left(4).offset =  pi; 
L_left(5) = Link([0 -lf 0    pi/2]);  L_left(5).offset = -pi/2; 
L_left(6) = Link([0 0   -lw -pi/2]);  L_left(6).offset = -pi/2;
L_left(7) = Link([0 0   lt   0]);
AllexLeft = SerialLink(L_left, 'name', 'ALLEX_{left}');

%Base HT, Tool HT
AllexLeft.base = [0              1      0              0;
                  -sin(15*D2R)   0     cos(15*D2R)     0;
                  cos(15*D2R)    0      sin(15*D2R)    lb;
                  0   0                0               1 ];

AllexLeft.tool = [ 1  0  0   0;
                   0  0  -1  0;
                   0  1  0   0;
                   0  0  0   1 ];


%% Right Arm

%DH parameter
L_right(1) = Link([0 -lp 0   -pi/2]); 
L_right(2) = Link([0 0   0   -pi/2]);  L_right(2).offset = -75*D2R;
L_right(3) = Link([0 -lu lep -pi/2]);  L_right(3).offset = -pi/2;
L_right(4) = Link([0 0   ley -pi/2]);  L_right(4).offset =  pi; 
L_right(5) = Link([0 -lf 0    pi/2]);  L_right(5).offset = -pi/2; 
L_right(6) = Link([0 0   -lw -pi/2]);  L_right(6).offset = -pi/2;
L_right(7) = Link([0 0 lt 0]);
AllexRight = SerialLink(L_right, 'name', 'ALLEX_{right}');

%Base HT, Tool HT
AllexRight.base = [0           1     0              0;
                  sin(15*D2R)  0     cos(15*D2R)    0;
                  cos(15*D2R)  0    -sin(15*D2R)    lb;
                  0   0                0            1 ];

AllexRight.tool = [ 1  0  0   0;
                    0  0  -1  0;
                    0  1  0   0;
                    0  0  0   1 ];

clear lb lp lu lep ley lf lw lt L_right L_left;

qz = [0 0 0 0 0 0 0]; % zero angles
qn = [0 0 0 -90 0 0 0]*D2R; % L shaped ready pose
qnr = [0 -5  10 -20 0 0 20]*D2R; % natural ready pose of left arm
qnl = [0  5 -10 -20 0 0 20]*D2R; % natural ready pose of right arm


% AllexLeft.plot(qnl,'scale',0.8, 'view', [120 20]); hold on; 
% AllexRight.plot(qnr,'scale',0.8, 'view', [120 20]);
