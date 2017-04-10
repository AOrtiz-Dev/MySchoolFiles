clear all
close all
clc


%% *******************************DAQ Setup********************************
s = daq.createSession('ni');
s.addAnalogInputChannel('myDAQ1', 'ai0', 'Voltage');
s.addAnalogOutputChannel('myDAQ1', 'ao0', 'Voltage');
s.Rate = (200000);


%% ***************************Initial Controller***************************
uUp = 8;	%Do not change this! This is the failsafe of the controller



u = 5.5;	%Initial controller voltage. Anything below 8 is safe
VR = 2.1013;     %Desired sensor voltage. Sensor range between #########Upper and Lower limits#########
Avg = 5;    %Moving average filter. Set to 1 to disable. Too high, and the system will act poorly, but at no harm.
M=500;      %Length of run





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%											%
% Define your universe and fuzzy sets here. %
%											%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
D = 0.9

err_x = [-0.9, 0.01, 0.9];

NB = trapmf(err_x, [-D -D -.5 -.45]);
NS = trimf(err_x,[-0.45 -0.325 -0.2]);
Z = trimf(err_x, [-0.2 0 0.2]);
PS = trimf(err_x, [0.2 0.325 0.45]);
PB = trapmf(err_x, [.45 .5 D D]);



%% ************************Fuzzy Controller Loop***************************
for i=1:M;
	VA(1) = s.inputSingleScan(); %Record voltage from sensor
	
    
    
    %---------------------------------------------------------------------
    %---------------       Moving Average Filter        ------------------
    %---------------------------------------------------------------------
    
    VAvg=VA(1);
    if (Avg > 1);
        if (i==1);
            for j=1:Avg;
               VA(j)=VA(1);
            end;
        else
            VAvg=VA(1);
            for j=2:Avg;
                VAvg=VAvg+VA(j);
            end;
            VAvg=VAvg/(Avg); %Average of last %Avg% voltages.
        end;
    end;
    
    
    
    
    
    
	e=VR-VAvg; %Calculate error
	
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%											%
	%   Define your rules and compute du here.  %
	%											%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
    DOF = interp1(err_x, NB, e);
    DU1 = min(NB, DOF);
    DOF = interp1(err_x, NS, e);
    DU2 = min(NS, DOF);
    DOF = interp1(err_x, Z, e);
    DU3 = min(Z, DOF);
    DOF = interp1(err_x, PS, e);
    DU4 = min(PS, DOF);
    DOF = interp1(err_x, PB, e);
    DU5 = min(PB, DOF);
    DU = max(max(max(max(DU1,DU2), DU3), DU4), DU5);
    du = 0.25*defuzz(err_x,DU,'centroid'); 
    
	u=u+du;
	
	if (u>uUp);		%Do not remove or modify this! 
		u=uUp;		%This protects the device from the controller 
	end;			%running out of control.
	
	s.outputSingleScan(u); %Set fan voltage to u
	
	PV(i)=VAvg; %Record voltage to array (Previous Voltage)
	Pu(i)=u; %Record controller to array (Previous u)
	
    
    
	if(Avg > 1) %Average moving window loop
		for j=0:Avg-2;
			VA(Avg-j)=VA(Avg-j-1);
		end;
    end;
    
    %---------------------------------------------------------------------
    %---------------           Notifications            ------------------
    %---------------------------------------------------------------------
    
    disp(strcat(num2str(i), '...',num2str(e), '...',num2str(u)))
    
    VL(i)=VR; %Stupid workaround to graph VR
    
    plot (1:i, PV(1:i), 'r', 1:i, VL(1:i), 'g');
    axis([0 M VR-.25 VR+.25]);
    drawnow
    
    
end;

s.outputSingleScan(0);	%Stops fan





