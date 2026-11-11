function plot_data(dataset_name)
close all
var_num = dataset_name.numElements;
for i = 1:var_num
    figure(); %create new figure for each plot
    plot(dataset_name{i}.Values.Time,dataset_name{i}.Values.Data);%plot the data

    %get the name of the variable being plotted 
    name_string = dataset_name{i}.BlockPath.getBlock(1);
    name = name_string(26:end);
    title(name); % add title to the plot
    xlabel("t(s)"); %add x axis label 
    ylabel(name); %add y axis label

end