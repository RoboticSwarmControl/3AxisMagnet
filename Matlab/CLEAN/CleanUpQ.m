%% Cleaning Function
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
function [ Task ] = CleanUpQ(soap)
%Print Task Name
Task = 'Clearing Figures or Data';
%---------------------
% Clean Data soap = 1;
% Clean Up Figures soap = 2;
%   
% CleanUpQ(soap)
% 
% EX__
% CleanUpQ(1);   
%}
%% CleanUpQ
% Enough Inputs EXCEPTION
if nargin == 1
    % If soap unavailable
    if soap ~=1 && soap~=2 && soap~=3
        display('Soap Not Available\n chose 1(Data) or 2(Figures)\n');
    end
    
    % Cleaning DATA
    if soap ==1
        % Set Up Q as not 0 or 1
        Q = NaN;
        while Q ~=0 && Q~=1 
            % Prompt Text
            prompt = 'Clear All DATA?\n 1-Yes or 0-No\n';
            % Call for you to input
            Q = input(prompt);
            % If yes close all figures
            if Q==1
                % Clear Data
                clear all;
                % Clear Output Window
                clc;
                Q = 1;
                soap = 1;
            end
        end
    end
    
    % Cleaning Figures
    if soap ==2
        % Set Up Q as not 0 or 1
        Q = NaN;
        while Q ~=0 && Q~=1
            % Prompt Text
            prompt = 'Clear all Figures?\n 1-Yes or 0-No\n';
            % Call for you to input
            Q = input(prompt);
            % If yes close all figures
            if Q==1
                % Close all windows
                close all;
            end
        end
    end
    
    % Cleaning Figures & DATA "DEAP CLEAN"
    if soap ==3
        % Set Up Q as =~(0|1)
        Q = NaN;
        while Q ~=0 && Q~=1
            % Prompt Text
            prompt = 'Clear Everything?\n 1-Yes or 0-No\n';
            % Call for you to input
            Q = input(prompt);
            % If yes close all figures
            if Q==1
                % Close all windows
                close all;
                % Clear Data
                clear all;
                % Clear Output Window
                clc;
                % To Excape While Loop
                Q = 1;
            end
        end
    end
    
else
    display('ERROR: Not Enough Input Arguments')
end
end

