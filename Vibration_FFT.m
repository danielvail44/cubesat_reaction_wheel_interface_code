% Script to read accelerometer data from serial port, 
% perform FFT and plot frequency content in real-time

clear all;
close all;
clc;

%% Configuration Parameters - ADJUST THESE TO MATCH YOUR SETUP
comPort = 'COM11';        % Serial port name (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
baudRate = 115200;       % Baud rate
bufferSize = 8192;       % Buffer size for FFT (power of 2 for efficiency)
samplingRate = 5000;     % Expected sampling rate in Hz 
maxG = 3;                % Accelerometer range in g (+/-)
adcResolution = 12;      % ADC resolution in bits
adcOffset = 2048;        % Value that corresponds to 0g (typically 2^(adcResolution-1))
                        
% Derived parameters
maxADCValue = 2^adcResolution - 1;  % Maximum ADC value (4095 for 12-bit)
scaleFactor = (2 * maxG) / maxADCValue;  % Scale factor to convert ADC values to g

% Flag for stopping acquisition (make it global so callback can access it)
global stopAcquisition;
stopAcquisition = false;

%% Initialize Serial Connection
try
    % Create and configure serial port object
    s = serialport(comPort, baudRate);
    % Set terminator based on your data format
    configureTerminator(s, "LF");  % Change if needed (e.g., "CR/LF")
    
    % Flush input buffer
    flush(s);
    
    disp(['Successfully connected to ' comPort]);
catch
    error(['Could not connect to ' comPort '. Check the port name and connection.']);
end

%% Create Figure for Real-time Plotting
hFig = figure('Name', 'Accelerometer FFT Analysis', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 800]);

% Set up a way to gracefully exit with a key press
set(hFig, 'KeyPressFcn', @keyPressCallback);

% Initialize data buffer
dataBuffer = zeros(1, bufferSize);
timeVector = (0:bufferSize-1) / samplingRate;
bufferIndex = 1;

% Calculate frequency axis for plotting (only positive frequencies)
freqAxis = samplingRate/2 * linspace(0, 1, bufferSize/2+1);

% Create plot handles
subplot(2,1,1);
timePlotHandle = plot(timeVector, dataBuffer);
title('Time Domain Signal');
xlabel('Time (s)');
ylabel('Acceleration (g)');
ylim([-maxG maxG]);
grid on;

subplot(2,1,2);
fftPlotHandle = plot(freqAxis, zeros(1, bufferSize/2+1));
title('Frequency Domain (FFT)');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
grid on;
xlim([0, samplingRate/2]);  % Limit to Nyquist frequency

% Add peak detection text annotation
peakText = annotation('textbox', [0.15, 0.35, 0.3, 0.1], 'String', 'Dominant Frequencies: N/A', ...
    'EdgeColor', 'none', 'BackgroundColor', [1 1 1 0.7]);

%% Main Loop for Continuous Data Acquisition and Processing
disp('Starting data acquisition. Press any key in the figure to stop.');
try
    tic;  % Start timer for sampling rate calculation
    sampleCount = 0;
    
    while ~stopAcquisition
        % Check if data is available
        if s.NumBytesAvailable > 0
            % Read a line of data
            rawData = readline(s);
            
            % Convert string to number
            try
                adcValue = str2double(rawData);
                
                % Make sure it's a valid ADC value
                if ~isnan(adcValue) && adcValue >= 0 && adcValue <= maxADCValue
                    % Convert ADC value to acceleration in g
                    % For 12-bit ADC mapping:
                    % 0     -> -3g
                    % 2048  -> 0g  (adcOffset)
                    % 4095  -> +3g
                    acceleration = (adcValue - adcOffset) * scaleFactor;
                    
                    % Add to buffer
                    dataBuffer(bufferIndex) = acceleration;
                    bufferIndex = bufferIndex + 1;
                    sampleCount = sampleCount + 1;
                    
                    % If buffer is full, perform FFT and update plots
                    if bufferIndex > bufferSize
                        % Reset buffer index
                        bufferIndex = 1;
                        
                        % Calculate actual sampling rate
                        actualSampleRate = samplingRate;
                        sampleCount = 0;
                        tic;
                        
                        % Update time vector and frequency axis based on actual sample rate
                        timeVector = (0:bufferSize-1) / actualSampleRate;
                        freqAxis = actualSampleRate/2 * linspace(0, 1, bufferSize/2+1);
                        
                        % Apply window function to reduce spectral leakage
                        windowedData = dataBuffer .* hann(bufferSize)';
                        
                        % Perform FFT
                        fftData = fft(windowedData);
                        
                        % Compute single-sided amplitude spectrum
                        P2 = abs(fftData/bufferSize);
                        P1 = P2(1:bufferSize/2+1);
                        P1(2:end-1) = 2*P1(2:end-1);  % Multiply by 2 (except DC and Nyquist)
                        
                        % Find dominant frequencies (peaks)
                        % Ignore DC component (first element)
                        [peaks, peakIndices] = findpeaks(P1(2:end), 'SortStr', 'descend', 'NPeaks', 3);
                        peakIndices = peakIndices + 1;  % Adjust indices for ignored DC component
                        
                        if ~isempty(peaks)
                            peakFreqs = freqAxis(peakIndices);
                            peakStr = sprintf('Dominant Frequencies: %.1f Hz (%.3f), %.1f Hz (%.3f), %.1f Hz (%.3f)', ...
                                [peakFreqs(1:min(3,length(peakFreqs))); peaks(1:min(3,length(peaks)))]);
                            set(peakText, 'String', peakStr);
                        else
                            set(peakText, 'String', 'Dominant Frequencies: N/A');
                        end
                        
                        % Update plots
                        set(timePlotHandle, 'XData', timeVector, 'YData', dataBuffer);
                        set(fftPlotHandle, 'XData', freqAxis, 'YData', P1);
                        
                        % Update titles with sampling rate info
                        subplot(2,1,1);
                        title(['Time Domain Signal (Sampling Rate: ' num2str(round(actualSampleRate)) ' Hz)']);
                        
                        subplot(2,1,2);
                        title(['Frequency Domain (FFT) - Nyquist Freq: ' num2str(round(actualSampleRate/2)) ' Hz']);
                        xlim([0, 1000]);  % Update frequency axis limit
                        
                        drawnow;
                    end
                else
                    warning('Invalid ADC value: %d', adcValue);
                end
            catch
                warning('Error parsing data: %s', rawData);
            end
        else
            % Small pause to prevent CPU hogging
            pause(0.001);
        end
    end
catch ME
    disp('Error in data acquisition:');
    disp(ME.message);
end

%% Clean up
clear s;
disp('Data acquisition stopped.');

% Define the key press callback function
function keyPressCallback(~, ~)
    global stopAcquisition;
    stopAcquisition = true;
end