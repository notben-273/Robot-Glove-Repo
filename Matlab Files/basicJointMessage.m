%% Connect to ROS2
setenv('ROS_DOMAIN_ID','93')
node = ros2node("matlab_node");
pub = ros2publisher(node, "joint_states", "sensor_msgs/JointState");
msg = ros2message(pub);

%% Create message
msg.name = { ...
    'thumb_joint0','thumb_joint1','thumb_joint2','thumb_joint3', ...
    'pointer_joint0','pointer_joint1','pointer_joint2','pointer_joint3', ...
    'middle_joint0','middle_joint1','middle_joint2','middle_joint3' };

% Angles in radians
msg.position = [ ...
    0, 0.3, 0.2, 0.1, ...      % thumb
    0, 0.5, 0.4, 0.2, ...      % pointer
    0, 0.4, 0.3, 0.15 ];       % middle

msg.velocity = [];
msg.effort   = [];

msg.header.stamp = ros2time(node,"now");

%% Sending the message
send(pub, msg);