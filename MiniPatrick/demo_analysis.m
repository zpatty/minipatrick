clear
close all
filename = 'test-output_14-04-2021_12:31:56.csv';
T = readtable(filename);
time = T.Var6;
time = time - time(1);
xloc = T.Var1;
yloc = T.Var2;
xgoal = T.Var4;
ygoal = T.Var5;
dist = sqrt((xloc - xgoal).^2 + (yloc - ygoal).^2);
plot(time(1:264), dist(1:264), 'b', 'LineWidth', 2)
set(gca, 'FontSize', 14);
%set(distfig,'Position',[100,100,500,350]);
%set(distfig,'PaperPosition',[1,1,5.8,3.5]);
ylabel('Distance to Goal (cm)');
xlabel('Time (sec)');


filename = 'test-output_14-04-2021_13:59:19.csv';
T = readtable(filename);
time = T.Var6;
time = time - time(1);
xloc = T.Var1;
yloc = T.Var2;
xgoal = T.Var4;
ygoal = T.Var5;
dist = sqrt((xloc - xgoal).^2 + (yloc - ygoal).^2);
figure
plot(time, dist, 'b', 'LineWidth', 2)
set(gca, 'FontSize', 14);
ylabel('Distance to Goal (cm)');
xlabel('Time (sec)');