clear all
close all
means_100 = [];
stds_100 = [];
means_200 = [];
stds_200 = [];
filelist = dir('*.csv');
for i = 1:length(filelist)
    filename = filelist(i).name;
    %filename = 'deformation-test-output_21-05-2021_16:45:45.csv';
    M = readmatrix(filename);
    X1 = M(:,1);
    X2 = M(:,3);
    Y1 = M(:,2);
    Y2 = M(:,4);
    t = M(:,5);
    t = t - t(1);
    theta = atand((Y1 - Y2)./(X1 - X2));
    [TF_min,P_min] = islocalmin(theta);
    [TF_max,P_max] = islocalmax(theta);
    
    count = 1;
    
    
    thresh = 0.15*max(P_min);
    valleys = P_min > thresh;
    peaks = P_max > thresh;
    
    vllys = theta(valleys);
    pks = theta(peaks);
    vllys = theta(valleys);
    pks = theta(peaks);
    %figure
    %plot(t,theta,t(valleys), vllys,'r*', t(peaks), pks, 'or')
    IS_EVEN = ~mod(i,2);
    if IS_EVEN
        means_200(end+1) = mean(theta);
        stds_200(end+1) = std(theta);
    else
        means_100(end+1) = mean(theta);
        stds_100(end+1) = std(theta);
    end
    
end
figure
errorbar([1 2 3 4],means_100, stds_100,'o', 'MarkerEdgeColor','red','MarkerFaceColor','red', 'Color', 'red')
hold on
errorbar([1 2 3 4],means_200, stds_200,'o', 'MarkerEdgeColor','blue','MarkerFaceColor','blue', 'Color', 'blue')
%set(gcf, 'Position',  [100, 100, 500, 400])
set(gca,'FontSize',14)
%xlim([1 349])
xlabel('Sample No', 'FontSize', 16);
ylabel('Angular Displacement', 'FontSize', 16);
legend('100 ms', '200 ms');
