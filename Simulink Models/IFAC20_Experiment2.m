% %------------C1---------------------
global max_error_signal;
warning('off','all');
%
results = zeros(5,11);
motor_results = zeros(5,11);
wind_results = zeros(5,11);
%
%
%
% %set up Quadcopter sim
% load('../Initial Conditions/Hover.mat');
% load('../Path Command Files/Path_Diamond.mat');
% load('../Quadcopter Structure Files/quadModel_+.mat');
%
%
% % %----------Agent--------------------

obsBus = Simulink.Bus();

obs(1) = Simulink.BusElement;
obs(1).Name = 'PhiC1';
obs(2) = Simulink.BusElement;
obs(2).Name = 'PhiC2';

obs(3) = Simulink.BusElement;
obs(3).Name = 'TheC1';
obs(4) = Simulink.BusElement;
obs(4).Name = 'TheC2';

obs(5) = Simulink.BusElement;
obs(5).Name = 'X_Error';
obs(6) = Simulink.BusElement;
obs(6).Name = 'Y_Error';

obsBus.Elements = obs;
%create observation bus object for quad sim

actInfo = rlNumericSpec([4 1]);
actInfo.Name = 'action';
actInfo.LowerLimit = zeros(4,1);
actInfo.UpperLimit =  ones(4,1); 

criticOutputInfo = rlNumericSpec([4 1]);
criticOutputInfo.Name = 'criticOutputInfo';

mdl = 'IFAC20_Random_Attitude_Error_Learning_2';

%open_system(mdl)
% 
% load('../Initial Conditions/Hover.mat');
% load('../Path Command Files/Path_Diamond.mat');
% load('../Quadcopter Structure Files/quadModel_+.mat');


agentBlk = [mdl '/Attitude Controller/RL Agent'];

obsInfo = bus2RLSpec('obsBus','Model',mdl);
%obsInfo.Name = 'observations';

env = rlSimulinkEnv(mdl,agentBlk,obsInfo,actInfo);

%load trained agent
load('agent.mat');
%load('Agent1_1500.mat');

%Training optionsrlDDPGAgentOptions
maxepisodes = 1;
maxsteps = ceil(30/0.01);

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

row = 1;
num_iter = 5;
motor_trigger = 1.1;
wind_trigger = 1.1;
for test_cases = 2:3
    if(test_cases == 2)
        motor_trigger = 0.9;
        wind_trigger = 1.1;
    elseif(test_cases == 3)
        motor_trigger = 1.1;
        wind_trigger = 0.9;
    end
    
    
    agentAvg = 0;
    for i=1:num_iter
        trainingStats = train(agent,env,trainOpts);
        outcome = trainingStats.SimulationInfo.max_error_signal;
        disp(outcome);
        agentAvg = agentAvg + outcome;
        if(test_cases == 2)
            motor_results(row,i) = outcome;
        elseif(test_cases ==3)
            wind_results(row,i) = outcome;
        else
            results(row,i) = outcome;
            
        end
        
    end
    disp("Agent Avg :");
    agentAvg = agentAvg/num_iter;
    disp(agentAvg);
    if(test_cases == 2)
        motor_results(row,num_iter+1) = agentAvg;
    elseif(test_cases ==3)
        wind_results(row,num_iter+1) = agentAvg;
    else
        results(row,num_iter+1) = agentAvg;
    end

% %-----------------------C1-------------    
    
    avgC1 = 0;
    row = 2;
    for i = 1:num_iter
        
        sim('IFAC20_Random_Exp1_C1.slx');
        outcome = max_error_signal;
        avgC1 = avgC1 + outcome;
        if(test_cases == 2)
            motor_results(row,i) = outcome;
        elseif(test_cases ==3)
            wind_results(row,i) = outcome;
        else
            results(row,i) = outcome;
            break;
        end
        disp(outcome);
        
        
        
    end
    
    c1_average = avgC1/num_iter;
    disp("C1 Avg:");
    disp(c1_average);
    
    if(test_cases == 2)
        motor_results(row,num_iter+1) =c1_average;
    elseif(test_cases ==3)
        wind_results(row,num_iter+1) = c1_average;
    else
        results(row,num_iter+1) = c1_average;
    end
    %------------C2---------------------
    avgC2 = 0;
    row=3;
    for i = 1:num_iter
        
        sim('IFAC20_Random_Exp1_C2_2.slx');
        outcome = max_error_signal;
        avgC2 = avgC2 + outcome;
        if(test_cases == 2)
            motor_results(row,i) = outcome;
        elseif(test_cases == 3)
            wind_results(row,i) = outcome;
        else
            results(row,i) = outcome;
            break;
        end
        disp(outcome);
        
        
        
    end
    
    c2_average = avgC2/num_iter;
    disp("C2 avg:");
    disp(c2_average);
    if(test_cases == 2)
        motor_results(row,num_iter+1) = c2_average;
    elseif(test_cases ==3)
        wind_results(row,num_iter+1) = c2_average;
    else
        results(row,num_iter+1) = c2_average;
    end
    %------------Pure Randomization----------------
    rand_avg = 0;
    row=4;
    for i = 1:num_iter
        
        sim('IFAC20_Random_Exp1_PureRand.slx');
        outcome = max_error_signal;
        rand_avg = rand_avg + outcome;
        if(test_cases == 2)
            motor_results(row,i) = outcome;
        elseif(test_cases == 3)
            wind_results(row,i) = outcome;
        else
            results(row,i) = outcome;
  
        end
        disp(outcome);
        
        
        
    end
     Rand_average = rand_avg/num_iter;
    
   disp("Rand Average:");
   disp(Rand_average);
 
    if(test_cases == 2)
        motor_results(row,num_iter+1) = Rand_average;
    elseif(test_cases ==3)
        wind_results(row,num_iter+1) = Rand_average;
    else
        results(row,num_iter+1) = Rand_average;
    end
%--------------------------------
    switch_avg = 0;
    row=5;
    for i = 1:num_iter
        
        sim('IFAC20_Random_Exp1_Switched.slx');
        outcome = max_error_signal;
        switch_avg = switch_avg + outcome;
        if(test_cases == 2)
            motor_results(row,i) = outcome;
        elseif(test_cases == 3)
            wind_results(row,i) = outcome;
        else
            results(row,i) = outcome;
  
        end
        disp(outcome);
        
        
        
    end
    if(test_cases == 2)
        motor_results(row,num_iter+1) = switch_avg/num_iter;
    elseif(test_cases ==3)
        wind_results(row,num_iter+1) = switch_avg/num_iter;
    else
        results(row,num_iter+1) = switch_avg/num_iter;
    end
   
    
    %------------------------------------
end
