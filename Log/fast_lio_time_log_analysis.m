clear
close all

Color_red = [0.6350 0.0780 0.1840];
Color_blue = [0 0.4470 0.7410];
Color_orange = [0.8500 0.3250 0.0980];
Color_green = [0.4660 0.6740 0.1880];
Color_lightblue = [0.3010 0.7450 0.9330];
Color_purple = [0.4940 0.1840 0.5560];
Color_yellow = [0.9290 0.6940 0.1250];

fast_lio_ikdtree = csvread("./fast_lio_time_log.csv",1,0);
timestamp_ikd = fast_lio_ikdtree(:,1);
timestamp_ikd = timestamp_ikd - min(timestamp_ikd);
total_time_ikd = fast_lio_ikdtree(:,2)*1e3;
scan_num = fast_lio_ikdtree(:,3);
incremental_time_ikd = fast_lio_ikdtree(:,4)*1e3;
search_time_ikd = fast_lio_ikdtree(:,5)*1e3;
delete_size_ikd = fast_lio_ikdtree(:,6);
delete_time_ikd = fast_lio_ikdtree(:,7) * 1e3;
tree_size_ikd_st = fast_lio_ikdtree(:,8);
tree_size_ikd = fast_lio_ikdtree(:,9);
add_points = fast_lio_ikdtree(:,10);

fast_lio_forest = csvread("fast_lio_time_log.csv",1,0);
fov_check_time_forest = fast_lio_forest(:,5)*1e3;
average_time_forest = fast_lio_forest(:,2)*1e3;
total_time_forest = fast_lio_forest(:,6)*1e3;
incremental_time_forest = fast_lio_forest(:,3)*1e3;
search_time_forest = fast_lio_forest(:,4)*1e3;
timestamp_forest = fast_lio_forest(:,1);

% Use slide window to calculate average
L = 1;     % Length of slide window
for i = 1:length(timestamp_ikd)
    if (i<L)
        average_time_ikd(i) = mean(total_time_ikd(1:i));
    else
        average_time_ikd(i) = mean(total_time_ikd(i-L+1:i));
    end
end
for i = 1:length(timestamp_forest)
    if (i<L)
        average_time_forest(i) = mean(total_time_forest(1:i));
    else
        average_time_forest(i) = mean(total_time_forest(i-L+1:i));
    end
end




f = figure;
set(gcf,'Position',[80 433 600 640])
tiled_handler = tiledlayout(3,1);
tiled_handler.TileSpacing = 'compact';
tiled_handler.Padding = 'compact';
nexttile;
hold on;
set(gca,'FontSize',12,'FontName','Times New Roman') 
plot(timestamp_ikd, average_time_ikd,'-','Color',Color_blue,'LineWidth',1.2);
plot(timestamp_forest, average_time_forest,'--','Color',Color_orange,'LineWidth',1.2);
lg = legend("ikd-Tree", "ikd-Forest",'location',[0.1314 0.8559 0.2650 0.0789],'fontsize',14,'fontname','Times New Roman')
title("Time Performance on FAST-LIO",'FontSize',16,'FontName','Times New Roman')
xlabel("time/s",'FontSize',16,'FontName','Times New Roman')
yl = ylabel("Run Time/ms",'FontSize',15,'Position',[285.7 5.5000 -1]);
xlim([32,390]);
ylim([0,23]);
ax1 = gca;
ax1.YAxis.FontSize = 12;
ax1.XAxis.FontSize = 12;
grid on
box on
% print('./Figures/fastlio_exp_average','-depsc','-r600')


index_ikd = find(search_time_ikd > 0);
search_time_ikd = search_time_ikd(index_ikd);
index_forest = find(search_time_forest > 0);
search_time_forest = search_time_forest(index_forest);

t = nexttile;
hold on;
boxplot_data_ikd = [incremental_time_ikd,total_time_ikd];
boxplot_data_forest = [incremental_time_forest,total_time_forest];
Colors_ikd = [Color_blue;Color_blue;Color_blue];
Colors_forest = [Color_orange;Color_orange;Color_orange];
% xticks([3,8,13])
h_search_ikd = boxplot(search_time_ikd,'Whisker',50,'Positions',1,'Colors',Color_blue,'Widths',0.3);
h_search_forest = boxplot(search_time_forest,'Whisker',50,'Positions',1.5,'Colors',Color_orange,'Widths',0.3);
h_ikd = boxplot(boxplot_data_ikd,'Whisker',50,'Positions',[3,5],'Colors',Color_blue,'Widths',0.3);
h_forest = boxplot(boxplot_data_forest,'Whisker',50,'Positions',[3.5,5.5],'Colors',Color_orange,'Widths',0.3);
ax2 = gca;
ax2.YAxis.Scale = 'log';
xlim([0.5,6.0])
ylim([0.0008,100])
xticks([1.25 3.25 5.25])
xticklabels({'Nearest Search','    Incremental Updates','Total Time'});
yticks([1e-3,1e-2,1e-1,1e0,1e1,1e2])
ax2.YAxis.FontSize = 12;
ax2.XAxis.FontSize = 14.5;
% ax.XAxis.FontWeight = 'bold';
ylabel('Run Time/ms','FontSize',14,'FontName','Times New Roman')
box_vars = [findall(h_search_ikd,'Tag','Box');findall(h_ikd,'Tag','Box');findall(h_search_forest,'Tag','Box');findall(h_forest,'Tag','Box')];
for j=1:length(box_vars)
    if (j<=3)
        Color = Color_blue;
    else
        Color = Color_orange;
    end
    patch(get(box_vars(j),'XData'),get(box_vars(j),'YData'),Color,'FaceAlpha',0.25,'EdgeColor',Color);
end
Lg = legend(box_vars([1,4]), {'ikd-Tree','ikd-Forest'},'Location',[0.6707 0.4305 0.265 0.07891],'fontsize',14,'fontname','Times New Roman');
grid on
set(gca,'YMinorGrid','off')
nexttile;
hold on;
grid on;
box on;
set(gca,'FontSize',12,'FontName','Times New Roman') 
plot(timestamp_ikd, alpha_bal_ikd,'-','Color',Color_blue,'LineWidth',1.2);
plot(timestamp_ikd, alpha_del_ikd,'--','Color',Color_orange, 'LineWidth', 1.2);
plot(timestamp_ikd, 0.6*ones(size(alpha_bal_ikd)), ':','Color','black','LineWidth',1.2);
lg = legend("\alpha_{bal}", "\alpha_{del}",'location',[0.7871 0.1131 0.1433 0.069],'fontsize',14,'fontname','Times New Roman')
title("Re-balancing Criterion",'FontSize',16,'FontName','Times New Roman')
xlabel("time/s",'FontSize',16,'FontName','Times New Roman')
yl = ylabel("\alpha",'FontSize',15, 'Position',[285.7 0.4250 -1])
xlim([32,390]);
ylim([0,0.85]);
ax3 = gca;
ax3.YAxis.FontSize = 12;
ax3.XAxis.FontSize = 12;
% print('./Figures/fastlio_exp_combine','-depsc','-r1200')
% exportgraphics(f,'./Figures/fastlio_exp_combine_1.pdf','ContentType','vector')

