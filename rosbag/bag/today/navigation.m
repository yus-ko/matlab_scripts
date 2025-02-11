addpath '../../'
if not(exist('robot'))
    clear;
    % nav.bagを適切なファイル名に変更
    % topic名を適切に変更
    robot = bag_open(["nav.bag"], ...
                    'TopicNameOdom',        ["/robot_0/odom", "/robot_1/odom", "/robot_2/odom"], ...
                    'TopicNameCmd',         ["/robot_0/cmd_vel", "/robot_1/cmd_vel"], ...
                    'TopicNamePath',        ["/robot_0/move_base/DWAPlannerROS/local_plan", "/robot_1/move_base/DWAPlannerROS/local_plan", "/robot_0/move_base/NavfnROS/plan"], ...
                    'TopicNamePotential',   ["/robot_0/move_base/potbot/potential_field/field/potential", "/robot_1/move_base/potbot/potential_field/field/potential"], ...
                    'TopicNameMap',         ["/robot_0/map"]);
end

robot_id = 1;

figure (1), clf;
hold on
show(robot{robot_id}.Map);
plot([robot{3}.Path(1).Path.X], [robot{3}.Path(1).Path.Y], "--", "DisplayName", "global path")
for i = 1:length(robot{robot_id}.Path)
    h=plot([robot{robot_id}.Path(i).Path.X], [robot{robot_id}.Path(i).Path.Y], "--", "DisplayName", "local path");
    if i ~= 1
        h.HandleVisibility = 'off';
    end
end
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

% apf_ui_animation(figure(3), robot, robot_id);