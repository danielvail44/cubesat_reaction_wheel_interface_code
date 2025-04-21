% David Taylor, 2024

% do not modify the next line
clc, clear all, close all

% indicate duration of experiment
experiment_duration_in_seconds = 10;

%% begin serial port session

port = serialport("COM6",115200);
flush(port);

%% run experiment using specified interval
T = 0.004;
N = 5000;
r = zeros(1,N);
% enter mode 1
write(port,'x',"char");

for num = 1:N
    if num < 1000
        r(num) = 2000;
    elseif num < 2500
        r(num) = -2000;
    else
        r(num) = 2000;
    end
    
    %write(port, 6, "int8");

end

for i = 1:N
    write(port, int2str(r(i)), "string");
    write(port, " ", "string");
end

%% newsection
% wait for experiment to complete
pause(.1);
flush(port)

% enter mode 0

% declare experimental data
u = NaN(1,N);
yexp = NaN(1,N);

pause(experiment_duration_in_seconds)
% collect experimental data
for i = 1:N
    u(i) = read(port,1,"int16");
    yexp(i) = read(port,1,"int16");
end

% define time grid

t = (0:N-1)*T;

%% plot task 1 results
%yexp = abs(yexp);
%r = abs(r);
subplot(3,1,1)
plot( t, yexp,t(1:length(t)-1), r(1:length(t)-1))
xlabel("Time (seconds)")
ylabel("Speed (RPM)")
%xlim([0 8])
grid on
subplot(3,1,2)
plot(t, movmean(u, 10))
ylim([-512 512])
xlabel("Time (seconds)")
ylabel("Command")
grid on
subplot(3,1,3);
plot(t(1:length(t)-1), 1/T*movmean(diff(yexp),300))
ylabel("Acceleration (RPM/s)")
ylim([-1000 1000])
grid on
%%
write(port,'b',"char");
pause(1);
write(port,'r',"char");

%% end serial port session

delete(port)
clear port