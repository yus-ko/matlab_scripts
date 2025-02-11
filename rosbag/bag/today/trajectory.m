addpath '../../'
if not(exist('robot'))
    clear;
    % traj.bagを適切なファイル名に変更
    robot = bag_open(["traj.bag"], ...
                    'TopicNameOdom',        ["/robot_0/odom"], ...
                    'TopicNameCmd',         ["/robot_0/cmd_vel"], ...
                    'TopicNamePath',        ["/robot_0/path"]);
end

robot_id = 1;

figure (1), clf;
hold on
n = length(robot{1}.Path);
plot([robot{robot_id}.Path(n).Path.X], [robot{robot_id}.Path(n).Path.Y], "DisplayName", "target path");
plot([robot{robot_id}.Odom.X], [robot{robot_id}.Odom.Y], "DisplayName", "trajectory")

legend show
xlabel("$X$ [m]","Interpreter","latex");
ylabel("$Y$ [m]","Interpreter","latex");
pbaspect([1 1 1]);
grid on

figure (2), clf;
subplot(3,1,1);
hold on
plot([robot{robot_id}.Command.Time], [robot{robot_id}.Command.V],   "DisplayName", "Command")
plot([robot{robot_id}.Odom.Time], [robot{robot_id}.Odom.V],         "DisplayName", "Measurement")
legend show
xlabel("time $t$ [s]", "Interpreter", "latex");
ylabel("linear $v$ [m/s]", "Interpreter", "latex");
grid on

subplot(3,1,2);
hold on
plot([robot{robot_id}.Command.Time], [robot{robot_id}.Command.Omega],   "DisplayName", "Command")
plot([robot{robot_id}.Odom.Time], [robot{robot_id}.Odom.Omega],         "DisplayName", "Mesurement")
xlabel("time $t$ [s]", "Interpreter", "latex");
ylabel("angular $\omega$ [rad/s]", "Interpreter", "latex");
grid on

subplot(3,1,3);
plot([robot{robot_id}.Odom.Time], [robot{robot_id}.Odom.Theta]);
xlabel("time $t$ [s]", "Interpreter", "latex");
ylabel("Yaw pose $\theta$ [rad]", "Interpreter", "latex");
grid on