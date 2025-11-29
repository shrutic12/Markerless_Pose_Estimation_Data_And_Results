%Load data and compute errors
path = "Warwick_robot_Tests_9.10/"
folders = ["4","9","205","411","425"]
mean_error = []

for i=1: length(folders)

    % Load aruco data
    file = path +"pt "+folders(i)+"/aruco_marker.bag";
    bag = rosbag(file);
    
    bSel = select(bag,'Topic','/Aruco_Marker');
    % 
    msgStructs = readMessages(bSel,'DataFormat','struct');
    x = cellfun(@(m) double(m.Pose.Position.X),msgStructs);
    y = cellfun(@(m) double(m.Pose.Position.Y),msgStructs);
    z = cellfun(@(m) double(m.Pose.Position.Z),msgStructs);
    aruco_position = [x,y,z];
   
    % Load point cloud data
    file =path + "pt "+folders(i)+"/pose_cloud.bag";
    bag = rosbag(file);

    bSel = select(bag,'Topic','/final_pose_output');
    % 
    msgStructs = readMessages(bSel,'DataFormat','struct');
    x = cellfun(@(m) double(m.Poses.Position.X),msgStructs);
    y = cellfun(@(m) double(m.Poses.Position.Y),msgStructs);
    z = cellfun(@(m) double(m.Poses.Position.Z),msgStructs);
    
    position_cloud = [x y z];

    % Compute sizes, take the largest subset sampled at the same time steps
    size_position_cloud = size(position_cloud,1);
    size_aruco_position = size(aruco_position,1);
    N = min(size_aruco_position,size_position_cloud);
    aruco_position = aruco_position(1:N,:);
    position_cloud = position_cloud(1:N,:);

    % Store the difference between readings in each component
    diff = position_cloud - aruco_position;
    diff_x = diff(:,1);
    diff_y = diff(:,2);
    diff_z = diff(:,3);

    % Compute Euclidean distance for each time step
    d = sqrt(diff_x.^2 + diff_y.^2 + diff_z.^2);

    % Store the mean error in mm
    mean_d_m= mean(d) ;
    mean_d_mm = mean(d)*1000;
    mean_error(i) = mean_d_mm;

    
end

disp("Mean of Mean errors in mm")
mean(mean_error)
disp("Mean errors in mm")
mean_error