clear all;
setenv("ROS_DOMAIN_ID","5");

% Create a ROS 2 node
node = ros2node('/example_node');

% Define the topic name and message type
topicName = '/points_raw';
msgType = 'sensor_msgs/PointCloud2';
outputTopicName = '/matlab_points_raw';

% Create a publisher for the output topic
pub = ros2publisher(node, outputTopicName, msgType);

% Create a subscriber for the input topic
sub = ros2subscriber(node, topicName, msgType, @(msg) pointCloudCallbacks(msg, pub));

% Define the callback function
function pointCloudCallbacks(msg, pub)
    % Read XYZ and intensity data from the message
    xyz = rosReadXYZ(msg);
    intensity = rosReadField(msg, 'intensity');
    
    % Create a point cloud object
    ptCloudIn = pointCloud(xyz, 'Intensity', intensity);
    tic;
    % Detect ISS features
    [~, indices] = detectISSFeatures(ptCloudIn,"Radius",1.0);
    
    % Extract the points and intensities of detected features
    pointsXYZ = xyz(indices, :);
    pointsIntensity = intensity(indices);
    
    % Create a new PointCloud2 message
    issMsg = ros2message('sensor_msgs/PointCloud2');
    issMsg.header.frame_id = 'velodyne';  % Replace with your desired frame ID
    issMsg.header.stamp = ros2time(now);     % Set the timestamp (current time)
    
    % Write XYZ and intensity data to the message
    issMsg = rosWriteXYZ(issMsg, pointsXYZ);
    issMsg = rosWriteIntensity(issMsg, pointsIntensity);
    
    % Publish the message
    send(pub, issMsg);
    elapsedTime = toc;
    % Print elapsed time for feature detection
    fprintf('Time to calculate ISS keypoints: %.4f seconds\n', elapsedTime);
end
