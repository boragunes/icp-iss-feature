clear all;
setenv("ROS_DOMAIN_ID","5");

% Create a ROS 2 node
node = ros2node('/example_node');

% Define the topic name and message type
topicName = '/velodyne_points';
msgType = 'sensor_msgs/PointCloud2';
outputTopicName = '/matlab_points_raw';
msgTypeTransform = 'geometry_msgs/TransformStamped';
outputTransformName = '/matlab_transform';

% Create a publisher for the output topic
pub = ros2publisher(node, outputTopicName, msgType);

pubTransform = ros2publisher(node, outputTransformName, msgTypeTransform);

% Create a subscriber for the input topic
sub = ros2subscriber(node, topicName, msgType, @(msg) pointCloudCallbacks(msg, pub, pubTransform));

% Define the callback function
function pointCloudCallbacks(msg, pub , pubTransform)
    % Read XYZ and intensity data from the message
    xyz = rosReadXYZ(msg);
    intensity = rosReadField(msg, 'intensity');
    
    % Create a point cloud object for the current frame
    ptCloudIn = pointCloud(xyz, 'Intensity', intensity);
    
    % Downsample the point cloud using a voxel grid filter
    gridSize = 0.5; % Adjust the voxel size as needed
    ptCloudIn = pcdenoise(ptCloudIn,"NumNeighbors",5,"Threshold",0.5);

    ptCloudIn = pcdownsample(ptCloudIn, 'gridAverage', gridSize);
    
    % Declare persistent variables to store the previous point cloud
    persistent prevPtCloud initialized;
    if isempty(initialized)
        initialized = false;
    end

    % If previous point cloud is not initialized, set the current point cloud as the previous one
    if ~initialized
        prevPtCloud = ptCloudIn;
        initialized = true;
        return;
    end
    
    % Perform FGR-based alignment with the previous point cloud
    tic;
    [tform, ~] = pcregisterfgr(ptCloudIn, prevPtCloud, 0.1,MaxIterations=100);
    
    % Print elapsed time for FGR registration
    transformMsg = ros2message('geometry_msgs/TransformStamped');
    transformMsg.header.stamp = ros2time(now);
    transformMsg.header.frame_id = 'map';
    transformMsg.header.child_frame_id = "matlab";
    transformMsg.transform.translation.x = double(tform.Translation(1));
    transformMsg.transform.translation.y = double(tform.Translation(2));
    transformMsg.transform.translation.z = double(tform.Translation(3));
    R = tform.Rotation;
    q = rotm2quat(R);
    transformMsg.transform.rotation.w = double(q(1));
    transformMsg.transform.rotation.x = double(q(2));
    transformMsg.transform.rotation.y = double(q(3));
    transformMsg.transform.rotation.z = double(q(4));
    
    % Create a new PointCloud2 message
    alignedMsg = ros2message('sensor_msgs/PointCloud2');
    alignedMsg.header.frame_id = 'velodyne';  % Replace with your desired frame ID
    alignedMsg.header.stamp = ros2time(now);  % Set the timestamp (current time)
    
    % Write XYZ and intensity data to the message
    alignedMsg = rosWriteXYZ(alignedMsg, ptCloudIn.Location);
    alignedMsg = rosWriteIntensity(alignedMsg, ptCloudIn.Intensity);
    
    % Publish the message
    send(pub, alignedMsg);
    send(pubTransform, transformMsg);
    
    % Update the previous point cloud for the next frame
    prevPtCloud = ptCloudIn;
end
