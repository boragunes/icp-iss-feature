clear all;
setenv("ROS_DOMAIN_ID","5");

% Create a ROS 2 node
node = ros2node('/example_node');

% Define the topic names and message types
topicName = '/velodyne_points';
msgType = 'sensor_msgs/PointCloud2';
outputTopicName = '/matlab_points_raw';
outputTransformName = '/matlab_transform';

% Create publishers for the output topics
pub = ros2publisher(node, outputTopicName, msgType);
pubTransform = ros2publisher(node, outputTransformName, 'std_msgs/Float64MultiArray');

% Create a subscriber for the input topic
sub = ros2subscriber(node, topicName, msgType, @(msg) pointCloudCallbacks(msg, pub, pubTransform));

% Define the callback function
function pointCloudCallbacks(msg, pub, pubTransform)
    % Read XYZ and intensity data from the message
    xyz = rosReadXYZ(msg);
    intensity = rosReadField(msg, 'intensity');
    
    % Create a point cloud object for the current frame
    ptCloudIn = pointCloud(xyz, 'Intensity', intensity);

    % Configure LiDAR parameters and process the point cloud
    params = lidarParameters("VLP16", 1800);
    ptCloudIn = pcorganize(ptCloudIn, params);
    ptCloudIn = pcdenoise(ptCloudIn, "NumNeighbors", 5, "Threshold", 0.5, "PreserveStructure", true);

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
    
    % Perform LOAM-based registration with the previous point cloud
    ptCloudInloam = detectLOAMFeatures(ptCloudIn);
    prevPtCloudloam = detectLOAMFeatures(prevPtCloud);

    tform = pcregisterloam(ptCloudInloam, prevPtCloudloam);
    alignedPtCloud = pctransform(ptCloudIn, tform);
    
    % Create a new PointCloud2 message for the aligned point cloud
    alignedMsg = ros2message('sensor_msgs/PointCloud2');
    alignedMsg.header.frame_id = 'velodyne';  % Replace with your desired frame ID
    alignedMsg.header.stamp = ros2time(now);  % Use the original timestamp from the incoming message
    
    % Write XYZ and intensity data to the message
    alignedMsg = rosWriteXYZ(alignedMsg, alignedPtCloud.Location);
    
    % Create a message to send the transformation matrix
    transformMsg = ros2message('std_msgs/Float64MultiArray');
    
    % Define dimensions and strides
    transformMsg.layout.dim(1).label = 'rows';
    transformMsg.layout.dim(1).size = uint32(4);
    transformMsg.layout.dim(1).stride = uint32(4 * 4); % Total number of elements in a row
    
    transformMsg.layout.dim(2).label = 'columns';
    transformMsg.layout.dim(2).size = uint32(4);
    transformMsg.layout.dim(2).stride = uint32(4);
    
    transformMsg.layout.data_offset = uint32(0); % No padding
    
    transformData = reshape(tform.T, 1, []);
    transformMsg.data = double(transformData);
    disp(transformData)% 4x4 matrix flattened to 16x1
    
    % Publish the aligned point cloud and transformation matrix

    send(pub, alignedMsg);
    send(pubTransform, transformMsg);
    
    % Update the previous point cloud for the next frame
    prevPtCloud = ptCloudIn;
end
