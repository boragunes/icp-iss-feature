clear all;
setenv("ROS_DOMAIN_ID","5");

% Create a ROS 2 node
node = ros2node('/example_node');

% Define the topic name and message types
topicName = '/points_raw';
msgType = 'sensor_msgs/PointCloud2';
msgTypeTransform = 'geometry_msgs/TransformStamped';
outputTopicName = '/matlab_points_raw';
outputTransformName = '/matlab_transform';

% Create publishers for the output topics
pub = ros2publisher(node, outputTopicName, msgType);
pubTransform = ros2publisher(node, outputTransformName, msgTypeTransform);

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
    ptCloudInloam = detectLOAMFeatures(ptCloudIn,"NumRegionsPerLaser",10,"MaxSharpEdgePoints",6,"MaxPlanarSurfacePoints",6);
    prevPtCloudloam = detectLOAMFeatures(prevPtCloud,"NumRegionsPerLaser",10,"MaxSharpEdgePoints",6,"MaxPlanarSurfacePoints",6);

    tform = pcregisterloam(ptCloudInloam, prevPtCloudloam,MaxIterations=50,Tolerance=[0.001,0.025],SearchRadius=10);
    
    % Create a new PointCloud2 message for the aligned point cloud
    alignedMsg = ros2message('sensor_msgs/PointCloud2');
    alignedMsg.header.frame_id = 'velodyne';  % Replace with your desired frame ID
    alignedMsg.header.stamp = ros2time(now);  % Use the original timestamp from the incoming message
    
    % Write XYZ and intensity data to the message
    alignedMsg = rosWriteXYZ(alignedMsg, ptCloudIn.Location);
    

    % Create a message to send the transformation matrix
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

    % Publish the aligned point cloud and transformation matrix
    send(pub, alignedMsg);
    send(pubTransform, transformMsg);
    % Update the previous point cloud for the next frame
    prevPtCloud = ptCloudIn;
end