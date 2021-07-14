filename = 'strain-output-30-06-2021_14:42:43.csv';
% detect and set import options
opts = delimitedTextImportOptions("Delimiter",{'"',','},...
    "ConsecutiveDelimitersRule","join",...
    "LeadingDelimitersRule","ignore",...
    "VariableNamesLine",0,...
    "NumVariables",2,...
    "ExtraColumnsRule","ignore");
% Set data types
opts = setvartype(opts,[1:2],'double');
% Import data
data = table2array(readtable(filename,opts));
time = linspace(0, 0.1*length(data), length(data));

plot(time, data(:,1), time, data(:,2), 'LineWidth', 3)
xlabel('Time (s)')
ylabel('Bend Angle (deg)')
legend('Angle 1', 'Angle 2')