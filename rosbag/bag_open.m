function robot = bag_open(bagFilePath, varargin)
    
    options.TopicNameOdom = ["/odom"];
    options.TopicNameCmd = ["/cmd_vel"];
    options.TopicNamePath = ["/Path"];
    options.TopicNameImu = ["/imu/data"];
    options.TopicNamePotential = ["/potbot/field/potential"];
    options.TopicNameModelPose = ["/gazebo/model_states"];
    options.TopicNameMap = ["/map"];
    options.FrameIds = {[]};
    
    if mod(numel(varargin), 2) ~= 0
        error('オプション引数はキーと値のペアでなければなりません。');
    end
    for i = 1:2:numel(varargin)
        options.(varargin{i}) = varargin{i+1};
    end
    
    fns = fieldnames(options);
    flen = length(options.(fns{1}));
    for i = 1:numel(fns)
        fn = fns{i};
        len = length(options.(fn));
        if len ~= flen
            for j = len+1:flen
                options.(fn)(j) = options.(fn)(len);
            end
        end
    end
    
    robot_idx = 1;
    for i = 1 : length(bagFilePath)
        bag = rosbag(bagFilePath(i));
        start_time = bag.StartTime;
        
        for j = 1 : length(options.TopicNameOdom)
    
            msg_structs = readMessages(select(bag, 'Topic', options.TopicNameOdom(j)),'DataFormat','struct');
            robot{robot_idx}.Odom = get_odom(msg_structs);
            robot{robot_idx}.Odom = arrayfun(@(m) setfield(m, 'Time', m.Time - start_time), robot{robot_idx}.Odom);

            msg_select = select(bag, 'Topic', options.TopicNameCmd(j));
            msg_structs = readMessages(msg_select,'DataFormat','struct');
            robot{robot_idx}.Command = get_command(msg_structs, msg_select);
            robot{robot_idx}.Command = arrayfun(@(m) setfield(m, 'Time', m.Time - start_time), robot{robot_idx}.Command);
            
            msg_structs = readMessages(select(bag, 'Topic', options.TopicNamePath(j)),'DataFormat','struct');
            robot{robot_idx}.Path = get_paths(msg_structs);
            robot{robot_idx}.Path = arrayfun(@(m) setfield(m, 'Time', m.Time - start_time), robot{robot_idx}.Path);

%             robot{robot_idx}.Tf = get_tf(bag);
%             robot{robot_idx}.Tf = arrayfun(@(m) setfield(m, 'Time', m.Time - start_time), robot{robot_idx}.Tf);

            msg_structs = readMessages(select(bag, 'Topic', options.TopicNameImu(j)),'DataFormat','struct');
            robot{robot_idx}.Imu = get_imu(msg_structs);
            robot{robot_idx}.Imu = arrayfun(@(m) setfield(m, 'Time', m.Time - start_time), robot{robot_idx}.Imu);
            
            msg_structs = readMessages(select(bag, 'Topic', options.TopicNamePotential(j)),'DataFormat','struct');
            robot{robot_idx}.Potential = get_potential(msg_structs);
            robot{robot_idx}.Potential = arrayfun(@(m) setfield(m, 'Time', m.Time - start_time), robot{robot_idx}.Potential);

            msg_structs = readMessages(select(bag, 'Topic', options.TopicNameMap(j)),'DataFormat','struct');
            if numel(msg_structs) > 1
                robot{robot_idx}.Map = rosReadOccupancyGrid(msg_structs{1,1});
            end
    
            robot_idx = robot_idx + 1;
    
        end
        
        msg_select = select(bag, 'Topic', options.TopicNameModelPose(1));
        msg_structs = readMessages(msg_select,'DataFormat','struct');
        % msg_structs = readMessages(select(bag, 'Topic', "/gazebo/model_states"),'DataFormat','struct');
        
        iniidx = robot_idx;
        for j = 1:length(msg_structs)

            for k = 1:length(msg_structs{j,1}.Name)
                robot{iniidx+k}.ModelName = msg_structs{j,1}.Name{k,1};
                robot{iniidx+k}.Odom.Time(j) = msg_select.MessageList.Time(j) - start_time;
                robot{iniidx+k}.Odom.X(j) = msg_structs{j,1}.Pose(k).Position.X;
                robot{iniidx+k}.Odom.Y(j) = msg_structs{j,1}.Pose(k).Position.Y;
                robot{iniidx+k}.Odom.Theta(j) = getyaw(msg_structs{j,1}.Pose(k).Orientation);
                robot{iniidx+k}.Odom.V(j) = msg_structs{j,1}.Twist(k).Linear.X;
                robot{iniidx+k}.Odom.Omega(j) = msg_structs{j,1}.Twist(k).Angular.Z;
            end

        end
        
%         Gazeboモデルを取得する場合は個々のコメントをはずす
%         for k = 1:length(msg_structs{j,1}.Name)
%             robot{iniidx+k}.Odom = arrayfun(@(t, x, y, theta, v, omega) ...
%                     struct('Time', t, 'X', x, 'Y', y, 'Theta', theta, 'V', v, 'Omega', omega), ...
%                     robot{iniidx+k}.Odom.Time, robot{iniidx+k}.Odom.X, robot{iniidx+k}.Odom.Y, robot{iniidx+k}.Odom.Theta, robot{iniidx+k}.Odom.V, robot{iniidx+k}.Odom.Omega);
%         end

        
        % model.Odom = get_odom(msg_structs);
        % model{robot_idx}.Odom = arrayfun(@(m) setfield(m, 'Time', m.Time - start_time), robot{robot_idx}.Odom);
        
        for j = 1 : length(options.FrameIds)
            if length(options.FrameIds{j}) == 2
                source = options.FrameIds{j}(1);
                target = options.FrameIds{j}(2);
                robot{robot_idx}.Frame = get_frame_pose(bag, source, target);
                robot_idx = robot_idx + 1;
            end
        end
        
    end

end

function paths = get_paths(path_msgs)
    t = cellfun(@(m) gettime(m.Header.Stamp), path_msgs);
    poses_arr = cellfun(@(m) m.Poses, path_msgs, 'UniformOutput', false);
    poses = cellfun(@(m) get_poses_struct(m), poses_arr, 'UniformOutput', false);
    paths = arrayfun(@(t, xyth) struct('Time', t, 'Path', xyth), t, poses);
end

function poses_struct = get_poses_struct(poses_arr)
    X = arrayfun(@(m) double(m.Pose.Position.X),poses_arr);
    Y = arrayfun(@(m) double(m.Pose.Position.Y),poses_arr);
    Theta = arrayfun(@(m) getyaw(m.Pose.Orientation),poses_arr);
    poses_struct = arrayfun(@(x, y, theta) ...
                    struct('X', x, 'Y', y, 'Theta', theta), ...
                    X, Y, Theta);
end

function robot = get_odom(msg_structs)
    Time = cellfun(@(m) gettime(m.Header.Stamp),msg_structs);
    X = cellfun(@(m) double(m.Pose.Pose.Position.X),msg_structs);
    Y = cellfun(@(m) double(m.Pose.Pose.Position.Y),msg_structs);
    Theta = cellfun(@(m) getyaw(m.Pose.Pose.Orientation),msg_structs);
    V = cellfun(@(m) double(m.Twist.Twist.Linear.X),msg_structs);
    Omega = cellfun(@(m) double(m.Twist.Twist.Angular.Z),msg_structs);
    robot = arrayfun(@(t, x, y, theta, v, omega) ...
                    struct('Time', t, 'X', x, 'Y', y, 'Theta', theta, 'V', v, 'Omega', omega), ...
                    Time, X, Y, Theta, V, Omega);
end

function robot = get_command(msg_structs, msg_select)
    if numel(msg_select.MessageList) == 0
        robot = [];
        return
    end

    Time = arrayfun(@(m) double(m.Time), table2struct(msg_select.MessageList));
    V = cellfun(@(m) double(m.Linear.X),msg_structs);
    Omega = cellfun(@(m) double(m.Angular.Z),msg_structs);
    robot = arrayfun(@(t, v, omega) ...
                    struct('Time', t, 'V', v, 'Omega', omega), ...
                    Time, V, Omega);
end

% function pose = get_tf(bag, source_frame, target_frame, tftime)
function pose = get_tf(bag)
    msg_structs = readMessages(select(bag, 'Topic', "/robot_5/odom"),'DataFormat','struct');
    
    i=1;
    for k = 1:numel(msg_structs)
        msg = msg_structs{k,1};
        if (canTransform(bag,"map", msg.ChildFrameId, msg.Header.Stamp))
            tf2 = getTransform(bag,"map", msg.ChildFrameId, msg.Header.Stamp);

            pose(i).Time = gettime(msg.Header.Stamp);
            pose(i).X = tf2.Transform.Translation.X;
            pose(i).Y = tf2.Transform.Translation.Y;
            pose(i).Theta = getyaw(tfquat2rosquat(tf2.Transform.Rotation));

            i=i+1;
        end
    end
    
end

function robot = get_imu(msg_structs)
    Time = cellfun(@(m) gettime(m.Header.Stamp),msg_structs);
    OmegaX = cellfun(@(m) double(m.AngularVelocity.X),msg_structs);
    OmegaY = cellfun(@(m) double(m.AngularVelocity.Y),msg_structs);
    OmegaZ = cellfun(@(m) double(m.AngularVelocity.Z),msg_structs);
    AccX = cellfun(@(m) double(m.LinearAcceleration.X),msg_structs);
    AccY = cellfun(@(m) double(m.LinearAcceleration.Y),msg_structs);
    AccZ = cellfun(@(m) double(m.LinearAcceleration.Z),msg_structs);
    Roll = cellfun(@(m) getroll(m.Orientation),msg_structs);
    Pitch = cellfun(@(m) getpitch(m.Orientation),msg_structs);
    Yaw = cellfun(@(m) getyaw(m.Orientation),msg_structs);
    robot = arrayfun(@(t, omegax, omegay, omegaz, accx, accy, accz, roll, pitch, yaw) ...
                    struct('Time', t,   'OmegaX', omegax, 'OmegaY', omegay, 'OmegaZ', omegaz,...
                                        'AccX', accx, 'AccY', accy, 'AccZ', accz,...
                                        'Roll', roll, 'Pitch', pitch, 'Yaw', yaw), ...
                    Time, OmegaX, OmegaY, OmegaZ, AccX, AccY, AccZ, Roll, Pitch, Yaw);
end

function robot = get_potential(msg_structs)
    Time = cellfun(@(m) gettime(m.Header.Stamp),msg_structs);
    PointCloud = cellfun(@(m) rosReadXYZ(m), msg_structs, 'UniformOutput', false);
    robot = arrayfun(@(t, xyz)  struct('Time', t, 'PointCloud', xyz), Time, PointCloud);
end

function pcl_struct = get_pcl_struct(pcl_arr)
    X = arrayfun(@(m) double(m(1,1)),pcl_arr);
    Y = arrayfun(@(m) double(m(2)),pcl_arr);
    Z = arrayfun(@(m) getyaw(m(3)),pcl_arr);
    pcl_struct = arrayfun(@(x, y, z) struct('X', x, 'Y', y, 'Z', z), X, Y, Z);
end

function psoes = get_frame_pose(bag, source_frame, target_frame)

    tf_bag = select(bag, 'Topic', '/tf');
    tf_msgs = readMessages(tf_bag, 'DataFormat', 'struct');

    psoes = [];
    pose_idx = 1;

    % メッセージの中から対応するフレームを探す
    for i = 1:length(tf_msgs)
        transforms = tf_msgs{i}.Transforms;
        for j = 1:length(transforms)
            if strcmp(transforms(j).ChildFrameId, target_frame)
                
                stamp = transforms(j).Header.Stamp;
                if (canTransform(bag, source_frame, target_frame, stamp))
                    tf2 = getTransform(bag, source_frame, target_frame, stamp);

                    translation = tf2.Transform.Translation;
                    rotation = tf2.Transform.Rotation;
    
                    rq = tfquat2rosquat(rotation);
                    [r,p,y] = rosquat2rpy(rq);
    
                    psoes(pose_idx).Time = gettime(stamp);
                    psoes(pose_idx).X = translation.X;
                    psoes(pose_idx).Y = translation.Y;
                    psoes(pose_idx).Theta = y;
                    pose_idx = pose_idx + 1;
                end
            end
        end
    end
end

function time = gettime(Stamp)
    time = double(Stamp.Sec) + double(Stamp.Nsec) / 1e9;
end

function rosquat = tfquat2rosquat(tfquat)
    rosquat = struct('MessageType', tfquat.MessageType, 'X', tfquat.X, 'Y', tfquat.Y, 'Z', tfquat.Z, 'W', tfquat.W);
end

function roll = getroll(quaternion)
    [roll,pitch,yaw] = rosquat2rpy(quaternion);
end

function pitch = getpitch(quaternion)
    [roll,pitch,yaw] = rosquat2rpy(quaternion);
end

function yaw = getyaw(quaternion)
    [roll,pitch,yaw] = rosquat2rpy(quaternion);
end

function [roll,pitch,yaw] = rosquat2rpy(quaternion)
    quat = rosReadQuaternion(quaternion);
    eul = quat2eul(quat);
    roll = eul(3);
    pitch = eul(2);
    yaw = eul(1);
end