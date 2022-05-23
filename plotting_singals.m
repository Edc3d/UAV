% plotting bag.files

%plotting position x,y,z
 bag = rosbag("pos.bag"); % name of bag
 ts = timeseries(bag) % create timeseries from data
 
 %%
 pos = ts.Data; % X,Y,Z position
 pos(:,5);
 figure()
 plot(pos(:,4:6))
 legend('X Position', 'Y Position', 'Z Position')
%x = [150 0 150 0];



 
 