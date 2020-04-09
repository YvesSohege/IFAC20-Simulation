%------------C1---------------------
global max_error_signal;
warning('off','all');

results = zeros(4,10);



%set up Quadcopter sim
load('../Initial Conditions/Hover.mat');
load('../Path Command Files/LargeDiamond.mat');
load('../Quadcopter Structure Files/quadModel_+.mat');


%----------Agent--------------------
obsBus = Simulink.Bus();

obs(1) = Simulink.BusElement;
obs(1).Name = 'RPM1';
obs(2) = Simulink.BusElement;
obs(2).Name = 'RPM2';

obs(3) = Simulink.BusElement;
obs(3).Name = 'RPM3';
obs(4) = Simulink.BusElement;
obs(4).Name = 'RPM4';

obs(5) = Simulink.BusElement;
obs(5).Name = 'x_err';
obs(6) = Simulink.BusElement;
obs(6).Name = 'y_err';

obs(7) = Simulink.BusElement;
obs(7).Name = 'PhiC1';
obs(8) = Simulink.BusElement;
obs(8).Name = 'PhiC2';

obs(9) = Simulink.BusElement;
obs(9).Name = 'TheC1';
obs(10) = Simulink.BusElement;
obs(10).Name = 'TheC2';

obsBus.Elements = obs;
%create observation bus object for quad sim

actInfo = rlNumericSpec([4 1]);
actInfo.Name = 'action';
actInfo.LowerLimit = zeros(4,1);
actInfo.UpperLimit =  ones(4,1); 

criticOutputInfo = rlNumericSpec([4 1]);
criticOutputInfo.Name = 'criticOutputInfo';
criticOutputInfo.LowerLimit = [-100];
criticOutputInfo.UpperLimit = [ 100];
% ADD LIMIT 0 and 1 (is it working?)


mdl = 'IFAC20_Random_Exp1_Agent';
%open_system(mdl)
load('../Initial Conditions/Hover.mat');
load('../Path Command Files/LargeDiamond.mat');
load('../Quadcopter Structure Files/quadModel_+.mat');


agentBlk = [mdl '/Attitude Controller/RL Agent'];

obsInfo = bus2RLSpec('obsBus','Model',mdl);
%obsInfo.Name = 'observations';

env = rlSimulinkEnv(mdl,agentBlk,obsInfo,actInfo);

%load trained agent
load('Agent2_3000.mat');
%load('Agent1_1500.mat');
load('Exp2 Actor.mat');
load('Exp2 Critic.mat');

%Training optionsrlDDPGAgentOptions
maxepisodes = 1;
maxsteps = ceil(15/0.01);

save_after_number_steps = 0;

trainOpts = rlTrainingOptions(... 
    'MaxEpisodes',maxepisodes, ...
    'MaxStepsPerEpisode',maxsteps, ...
    'StopOnError','off', ...
    'ScoreAveragingWindowLength',1, ...
    'Verbose', true, ...
    'Plots','training-progress',...
    'StopTrainingCriteria','EpisodeCount',...
    'StopTrainingValue',maxepisodes);

agentAvg = 0;
row = 1;
for i=1:10
    trainingStats = train(agent,env,trainOpts);
    outcome = trainingStats.SimulationInfo.max_error_signal;
    disp(outcome);
    agentAvg = agentAvg + outcome;
    results(row,i) = outcome;
    
end
disp("Agent Avg :");
disp(agentAvg/10);
%-----------------------C1-------------

avgC1 = 0;
row = 2;
for i = 1:10
    
    sim('IFAC20_Random_Exp1_C1.slx');
    outcome = max_error_signal;
    avgC1 =  outcome;
    results(row,i) = outcome;
    disp(outcome);
 
    
  
end

c1_average = avgC1/10;

%------------C2---------------------
avgC2 = 0;
row=3;
for i = 1:10
    
    sim('IFAC20_Random_Exp1_C2.slx');
    outcome = max_error_signal;
    avgC2 = outcome;
    results(row,i) = outcome;
    disp(outcome);
 
    
  
end

c2_average = avgC2/10;


%------------Pure Randomization----------------
rand_avg = 0;
row=4;
for i = 1:10
    
    sim('IFAC20_Random_Exp1_PureRand.slx');
    outcome = max_error_signal;
    rand_avg = outcome;
    results(row,i) = outcome;
    disp(outcome);
 
    
  
end

Rand_average = rand_avg/10;




%------------------------------------

