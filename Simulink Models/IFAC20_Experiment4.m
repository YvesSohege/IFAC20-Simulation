% %------------C1---------------------
global total_att_error;
global total_traj_error;
warning('off','all');
%


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
obs(5).Name = 'X_error';
obs(6) = Simulink.BusElement;
obs(6).Name = 'Y_error';

obs(7) = Simulink.BusElement;
obs(7).Name = 'Phi_Error';
obs(8) = Simulink.BusElement;
obs(8).Name = 'Theta_Error';

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


mdl = 'IFAC20_Random_Attitude_Error_Learning_3';

%open_system(mdl)

load('../Initial Conditions/Hover.mat');
load('../Path Command Files/LargeDiamond.mat');
load('../Quadcopter Structure Files/quadModel_+.mat');


agentBlk = [mdl '/Attitude Controller/RL Agent'];

obsInfo = bus2RLSpec('obsBus','Model',mdl);
%obsInfo.Name = 'observations';

env = rlSimulinkEnv(mdl,agentBlk,obsInfo,actInfo);


%load trained agent
%load('TrainedAgent3.mat');
%load('Agent1_1500.mat');

%Training optionsrlDDPGAgentOptions
maxepisodes = 1;
maxsteps = ceil(60/0.1);

save_after_number_steps = 0;

trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes, ...
    'MaxStepsPerEpisode',maxsteps, ...
    'StopOnError','off', ...
    'ScoreAveragingWindowLength',1, ...
    'Verbose', true, ...
    'Plots','none',...
    'StopTrainingCriteria','EpisodeCount',...
    'StopTrainingValue',maxepisodes);

row = 1;
num_iter = 1;

% Exp1T = zeros(5,num_iter+1);
% Exp2T = zeros(5,num_iter+1);
% Exp3T = zeros(5,num_iter+1);
% Exp4T = zeros(5,num_iter+1);
% Exp5T = zeros(5,num_iter+1);
% Exp6T = zeros(5,num_iter+1);
% Exp7T = zeros(5,num_iter+1);
% 
% Exp1A = zeros(5,num_iter+1);
% Exp2A = zeros(5,num_iter+1);
% Exp3A = zeros(5,num_iter+1);
% Exp4A = zeros(5,num_iter+1);
% Exp5A = zeros(5,num_iter+1);
% Exp6A = zeros(5,num_iter+1);
% Exp7A = zeros(5,num_iter+1);



motor_trigger = 1.1;
wind_trigger = 1.1;

noise_power = 0;
noise_power1 = 0;
noise_power_pos = 0;
noise_power_pos1 = 0;


for test_cases = 1:1
    disp("Test Case:");
    disp(test_cases);
    if(test_cases == 2)
        %rotor 20% atttiude 0.02
        motor_trigger = 0.8;
        wind_trigger = 1.1;
        noise_power = 0.02;
        noise_power1 = 0.02;
        noise_power_pos = 0;
        noise_power_pos1 = 0;
    elseif(test_cases == 3)
        motor_trigger = 0.7;
        wind_trigger = 1.1;
        noise_power = 0;
        noise_power1 = 0;
        noise_power_pos = 0;
        noise_power_pos1 = 0;
    elseif(test_cases == 4)
        motor_trigger = 1.1;
        wind_trigger = 1.1;
        noise_power = 0.05;
        noise_power1 = 0.05;
        noise_power_pos = 0;
        noise_power_pos1 = 0;
    elseif(test_cases == 5)
        motor_trigger = 1.1;
        wind_trigger = 0.7;
        noise_power = 0;
        noise_power1 = 0;
        noise_power_pos = 0;
        noise_power_pos1 = 0;
    elseif(test_cases == 6)
        motor_trigger = 1.1;
        wind_trigger = 1.1;
        noise_power = 0;
        noise_power1 = 0;
        noise_power_pos = 0.05;
        noise_power_pos1 = 0.05;
    elseif(test_cases == 7)
        motor_trigger = 0.9;
        wind_trigger = 0.9;
        noise_power = 0.02;
        noise_power1 = 0.02;
        noise_power_pos = 0.02;
        noise_power_pos1 = 0.02;
    end
    
    
     agentAvg_att = 0;
     agentAvg_traj = 0;
     for i=1:num_iter
  
        trainingStats = train(agent,env,trainOpts);
        att_err = trainingStats.SimulationInfo.total_att_error;
        traj_err = trainingStats.SimulationInfo.total_traj_error;
        
        agentAvg_att = agentAvg_att + att_err;
        agentAvg_traj = agentAvg_traj + traj_err;
        if(test_cases == 2)
            Exp2T(row,i) = traj_err;
            Exp2A(row,i) =  att_err;
        elseif(test_cases ==3)
            Exp3T(row,i) = traj_err;
            Exp3A(row,i) =  att_err;
        elseif(test_cases ==4)
            Exp4T(row,i) = traj_err;
            Exp4A(row,i) =  att_err;
        elseif(test_cases ==5)
            Exp5T(row,i) = traj_err;
            Exp5A(row,i) =  att_err;
        elseif(test_cases ==6)
            Exp6T(row,i) = traj_err;
            Exp6A(row,i) =  att_err;
        elseif(test_cases ==7)
            Exp7T(row,i) = traj_err;
            Exp7A(row,i) =  att_err;
        else
            Exp1T(row,i) = traj_err;
            Exp1A(row,i) =  att_err;
        end
        
        
        
    end
    
    agentAvg_att = agentAvg_att/num_iter;
    agentAvg_traj = agentAvg_traj/num_iter;
    disp("Agent:");
    disp(agentAvg_att);
    disp(agentAvg_traj);
    
    if(test_cases == 2)
       Exp2T(row,i+1) = agentAvg_traj;
       Exp2A(row,i+1) =  agentAvg_att;
    elseif(test_cases ==3)
        Exp3T(row,i+1) = agentAvg_traj;
       Exp3A(row,i+1) =  agentAvg_att;
    elseif(test_cases ==4)
        Exp4T(row,i+1) = agentAvg_traj;
       Exp4A(row,i+1) =  agentAvg_att;
    elseif(test_cases ==5)
        Exp5T(row,i+1) = agentAvg_traj;
       Exp5A(row,i+1) =  agentAvg_att;
    elseif(test_cases ==6)
        Exp6T(row,i+1) = agentAvg_traj;
       Exp6A(row,i+1) =  agentAvg_att;
    elseif(test_cases ==7)
        Exp7T(row,i+1) = agentAvg_traj;
       Exp7A(row,i+1) =  agentAvg_att;
    else
        Exp1T(row,i+1) = agentAvg_traj;
       Exp1A(row,i+1) =  agentAvg_att;
        
   end
%     % %-----------------------C1-------------
%     
%     c1Avg_att = 0;
%     c1Avg_traj = 0;
%     row = 2;
%     for i = 1:num_iter
%         
%         sim('AAAI20_Random_Exp1_C1.slx');
%         att_err = total_att_error;
%         traj_err = total_traj_error;
%        
%         c1Avg_att = c1Avg_att + att_err;
%         c1Avg_traj = c1Avg_traj + traj_err;
%         if(test_cases == 2)
%             Exp2T(row,i) = traj_err;
%             Exp2A(row,i) =  att_err;
%         elseif(test_cases ==3)
%             Exp3T(row,i) = traj_err;
%             Exp3A(row,i) =  att_err;
%         elseif(test_cases ==4)
%             Exp4T(row,i) = traj_err;
%             Exp4A(row,i) =  att_err;
%         elseif(test_cases ==5)
%             Exp5T(row,i) = traj_err;
%             Exp5A(row,i) =  att_err;
%         elseif(test_cases ==6)
%             Exp6T(row,i) = traj_err;
%             Exp6A(row,i) =  att_err;
%         elseif(test_cases ==7)
%             Exp7T(row,i) = traj_err;
%             Exp7A(row,i) =  att_err;
%         else
%             Exp1T(row,i) = traj_err;
%             Exp1A(row,i) =  att_err;
%         end
%         
%         
%         
%     end
%     
%     c1Avg_att = c1Avg_att/num_iter;
%     c1Avg_traj = c1Avg_traj/num_iter;
%     disp("C1:");
%     disp(c1Avg_att);
%     disp(c1Avg_traj);
%     
%     if(test_cases == 2)
%        Exp2T(row,i+1) = c1Avg_traj;
%        Exp2A(row,i+1) =  c1Avg_att;
%     elseif(test_cases ==3)
%         Exp3T(row,i+1) = c1Avg_traj;
%        Exp3A(row,i+1) =  c1Avg_att;
%     elseif(test_cases ==4)
%         Exp4T(row,i+1) = c1Avg_traj;
%        Exp4A(row,i+1) =  c1Avg_att;
%     elseif(test_cases ==5)
%         Exp5T(row,i+1) = c1Avg_traj;
%        Exp5A(row,i+1) =  c1Avg_att;
%     elseif(test_cases ==6)
%         Exp6T(row,i+1) = c1Avg_traj;
%        Exp6A(row,i+1) =  c1Avg_att;
%     elseif(test_cases ==7)
%         Exp7T(row,i+1) = c1Avg_traj;
%        Exp7A(row,i+1) =  c1Avg_att;
%     else
%         Exp1T(row,i+1) = c1Avg_traj;
%        Exp1A(row,i+1) =  c1Avg_att;
%         
%    end
%     %------------C2---------------------
%     avgC2_att = 0;
%     avgC2_traj = 0;
%     row=3;
%     for i = 1:num_iter
%         
%         sim('AAAI20_Random_Exp1_C2_2.slx');
%         att_err = total_att_error;
%         traj_err = total_traj_error;
%        
%         avgC2_att = avgC2_att + att_err;
%         avgC2_traj = avgC2_traj + traj_err;
%         if(test_cases == 2)
%             Exp2T(row,i) = traj_err;
%             Exp2A(row,i) =  att_err;
%         elseif(test_cases ==3)
%             Exp3T(row,i) = traj_err;
%             Exp3A(row,i) =  att_err;
%         elseif(test_cases ==4)
%             Exp4T(row,i) = traj_err;
%             Exp4A(row,i) =  att_err;
%         elseif(test_cases ==5)
%             Exp5T(row,i) = traj_err;
%             Exp5A(row,i) =  att_err;
%         elseif(test_cases ==6)
%             Exp6T(row,i) = traj_err;
%             Exp6A(row,i) =  att_err;
%         elseif(test_cases ==7)
%             Exp7T(row,i) = traj_err;
%             Exp7A(row,i) =  att_err;
%         else
%             Exp1T(row,i) = traj_err;
%             Exp1A(row,i) =  att_err;
%         end
%         
%         
%         
%     end
%     
%     avgC2_att = avgC2_att/num_iter;
%     avgC2_traj = avgC2_traj/num_iter;
%      disp("C2:");
%     disp(avgC2_att);
%     disp(avgC2_traj);
%     if(test_cases == 2)
%        Exp2T(row,i+1) = avgC2_traj;
%        Exp2A(row,i+1) =  avgC2_att;
%     elseif(test_cases ==3)
%         Exp3T(row,i+1) = avgC2_traj;
%        Exp3A(row,i+1) =  avgC2_att;
%     elseif(test_cases ==4)
%         Exp4T(row,i+1) = avgC2_traj;
%        Exp4A(row,i+1) =  avgC2_att;
%     elseif(test_cases ==5)
%         Exp5T(row,i+1) = avgC2_traj;
%        Exp5A(row,i+1) =  avgC2_att;
%     elseif(test_cases ==6)
%         Exp6T(row,i+1) = avgC2_traj;
%        Exp6A(row,i+1) =  avgC2_att;
%     elseif(test_cases ==7)
%         Exp7T(row,i+1) = avgC2_traj;
%        Exp7A(row,i+1) =  avgC2_att;
%     else
%         Exp1T(row,i+1) = avgC2_traj;
%        Exp1A(row,i+1) =  avgC2_att;
%         
%    end
%     ------------Pure Randomization----------------
%     avgRand_traj = 0;
%     avgRand_att = 0;
%     row=4;
%     for i = 1:num_iter
%         
%         sim('AAAI20_Random_Exp1_PureRand.slx');
%         att_err = total_att_error;
%         traj_err = total_traj_error;
%        
%         avgRand_att = avgRand_att + att_err;
%         avgRand_traj = avgRand_traj + traj_err;
%         if(test_cases == 2)
%             Exp2T(row,i) = traj_err;
%             Exp2A(row,i) =  att_err;
%         elseif(test_cases ==3)
%             Exp3T(row,i) = traj_err;
%             Exp3A(row,i) =  att_err;
%         elseif(test_cases ==4)
%             Exp4T(row,i) = traj_err;
%             Exp4A(row,i) =  att_err;
%         elseif(test_cases ==5)
%             Exp5T(row,i) = traj_err;
%             Exp5A(row,i) =  att_err;
%         elseif(test_cases ==6)
%             Exp6T(row,i) = traj_err;
%             Exp6A(row,i) =  att_err;
%         elseif(test_cases ==7)
%             Exp7T(row,i) = traj_err;
%             Exp7A(row,i) =  att_err;
%         else
%             Exp1T(row,i) = traj_err;
%             Exp1A(row,i) =  att_err;
%         end
%         
%         
%         
%     end
%     
%     avgRand_att = avgRand_att/num_iter;
%     avgRand_traj = avgRand_traj/num_iter;
%      disp("Rand:");
%     disp(avgRand_att);
%     disp(avgRand_traj);
%     if(test_cases == 2)
%        Exp2T(row,i+1) = avgRand_traj;
%        Exp2A(row,i+1) =  avgRand_att;
%     elseif(test_cases ==3)
%         Exp3T(row,i+1) = avgRand_traj;
%        Exp3A(row,i+1) =  avgRand_att;
%     elseif(test_cases ==4)
%         Exp4T(row,i+1) = avgRand_traj;
%        Exp4A(row,i+1) =  avgRand_att;
%     elseif(test_cases ==5)
%         Exp5T(row,i+1) = avgRand_traj;
%        Exp5A(row,i+1) =  avgRand_att;
%     elseif(test_cases ==6)
%         Exp6T(row,i+1) = avgRand_traj;
%        Exp6A(row,i+1) =  avgRand_att;
%     elseif(test_cases ==7)
%         Exp7T(row,i+1) = avgRand_traj;
%        Exp7A(row,i+1) =  avgRand_att;
%     else
%         Exp1T(row,i+1) = avgRand_traj;
%        Exp1A(row,i+1) =  avgRand_att;
%         
%    end
   
    %--------------------------------
%     avgSwitch_traj = 0;
%     avgSwitch_att = 0;
%     row=5;
%     for i = 1:num_iter
%         
%         sim('AAAI20_Random_Exp1_Switched.slx');
%                att_err = total_att_error;
%         traj_err = total_traj_error;
%        
%         avgSwitch_att = avgSwitch_att + att_err;
%         avgSwitch_traj = avgSwitch_traj + traj_err;
%         if(test_cases == 2)
%             Exp2T(row,i) = traj_err;
%             Exp2A(row,i) =  att_err;
%         elseif(test_cases ==3)
%             Exp3T(row,i) = traj_err;
%             Exp3A(row,i) =  att_err;
%         elseif(test_cases ==4)
%             Exp4T(row,i) = traj_err;
%             Exp4A(row,i) =  att_err;
%         elseif(test_cases ==5)
%             Exp5T(row,i) = traj_err;
%             Exp5A(row,i) =  att_err;
%         elseif(test_cases ==6)
%             Exp6T(row,i) = traj_err;
%             Exp6A(row,i) =  att_err;
%         elseif(test_cases ==7)
%             Exp7T(row,i) = traj_err;
%             Exp7A(row,i) =  att_err;
%         else
%             Exp1T(row,i) = traj_err;
%             Exp1A(row,i) =  att_err;
%         end
%         
%         
%         
%     end
%     
%     avgSwitch_att = avgSwitch_att/num_iter;
%     avgSwitch_traj = avgSwitch_traj/num_iter;
%      disp("Switched:");
%     disp(avgSwitch_att);
%     disp(avgSwitch_traj);
%     if(test_cases == 2)
%        Exp2T(row,i+1) = avgSwitch_traj;
%        Exp2A(row,i+1) =  avgSwitch_att;
%     elseif(test_cases ==3)
%         Exp3T(row,i+1) = avgSwitch_traj;
%        Exp3A(row,i+1) =  avgSwitch_att;
%     elseif(test_cases ==4)
%         Exp4T(row,i+1) = avgSwitch_traj;
%        Exp4A(row,i+1) =  avgSwitch_att;
%     elseif(test_cases ==5)
%         Exp5T(row,i+1) = avgSwitch_traj;
%        Exp5A(row,i+1) =  avgSwitch_att;
%     elseif(test_cases ==6)
%         Exp6T(row,i+1) = avgSwitch_traj;
%        Exp6A(row,i+1) =  avgSwitch_att;
%     elseif(test_cases ==7)
%         Exp7T(row,i+1) = avgSwitch_traj;
%        Exp7A(row,i+1) =  avgSwitch_att;
%     else
%         Exp1T(row,i+1) = avgSwitch_traj;
%        Exp1A(row,i+1) =  avgSwitch_att;
%         
%    end
%     
%     
    %------------------------------------
end
