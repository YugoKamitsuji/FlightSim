% aerodynamics related parameters
b =  1.5652e-08 * (60/(2*pi))^2 ;   % Thrust coeffcient intially N/(rpm^2), now N/(rev/s^2)
k =  2.0862e-10 * (60/(2*pi))^2 ;   % Torque Coeffcient intially Nm/(rpm^2), now Nm/(rev/s^2)
R=3*0.0254;                         % propeller radius  [m]
nb=2;                               % number of blade
A=pi*R^2;                           % disk area
% sig=nb*mean(c)/(pi*R);              % solidity

% blade characteristics
rho = 1.15;
nr = 20;                             % number of radial points on a blade
npsi = 60;                            % number of azimuthal points for 2*Pi

%the0=repmat(5,[1,11]);
%cla1=repmat(1.8059*pi,[1,11]);
the0=repmat(4,[1,11]);
cla1=repmat(1.7059*pi,[1,11]);
th1=(the0+[24.9844849214694,24.4885730384207,23.6542985258914,22.4816610238794,20.9706612541160,19.1212982695717,16.9335723826041,14.4074838117246,11.5430326339323,8.34021849685304,4.79904143902087])*pi/180;
c1=[7.96284784614477,11.2448599794330,13.6346682267195,15.1322722373498,15.7376722802413,15.4508682842241,14.2718602574625,12.2006482106621,9.23723208187088,5.38161196932697,0.633787695366624]*0.001;

% interpolating data for different radius location (final size of the vectors 1*nr)
th=interp1(linspace(1/11,1,11),th1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');
c=interp1(linspace(1/11,1,11),c1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');
cla=interp1(linspace(1/11,1,11),cla1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');

r=linspace(floor(nr*0.01)/nr,1,nr);    % normolized radial locations
psi=linspace(0,2*pi,npsi);             % azimuth angle

maxsize=max(nr,npsi);
numvar=11;
geometry2=zeros(numvar,maxsize);

list={R,nb,A,rho,nr,npsi,th,c,cla,r,psi};

for  i=1:numvar

    geometry2(i,1:length(list{i}))=[list{i}];

end

geometry = Simulink.Signal;
geometry .DataType = 'double';
geometry .Dimensions = [length(list) npsi];
geometry .Complexity = 'real';
geometry .SamplingMode = 'Sample based';
geometry .InitialValue = 'geometry2';
