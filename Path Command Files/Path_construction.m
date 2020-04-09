%% This demonstration script generates a timeseries in the workspace
%
X = [ 0, 10 , 0 ,-10,0  , 10, 0]; % meters
Y = [ 0, 0  ,10 , 0 ,-10, 0 , 0];  % meters
Z = [ 3, 3  , 3 , 3 ,3  , 3 , 3];  % meters
t = [ 0, 10 , 20, 30, 40, 50 , 60]; % seconds
Psi = [0, 0, 0, 0, 0, 0, 0]; % radians
path.x = timeseries(X,t);
path.y = timeseries(Y,t);
path.z = timeseries(Z,t);
path.psi = timeseries(Psi,t);
clear X Y Z t Psi
uisave('path', 'LargeDiamond')
%clear path