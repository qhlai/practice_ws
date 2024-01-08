
clear all
close all

# sudo apt install octave-signal
pkg load signal

# Use with files generated by export_stats.sh

dataDir = 'SET_PATH_TO_RESULTS_DIR';
resultsToShow = 1; % 1=single 2=Consecutive

prefix = 'Stat'; 
RAMaddOverhead = 0;
% Inliers_ratio = 'Loop/Visual_inliers/' ./ 'Keypoint/Current_frame/words'
% Odometry_average = 'Memory/Distance_travelled/m'(2:end) - 'Memory/Distance_travelled/m'(1:end-1)

statNames = {'Loop/Odom_correction_norm/m', 'Loop/Visual_inliers/', 'Inliers_ratio_%', 'Timing/Total/ms', 'Memory/RAM_usage/MB', 'Memory/RAM_estimated/MB', 'Keypoint/Current_frame/words', 'Loop/Map_id/', 'Memory/Local_graph_size/', 'Keypoint/Dictionary_size/words', 'Loop/Distance_since_last_loc/'}; % 'Odometry_average'

 
datasets = [ 0 1 6 7 9 14 11 111 ]; % 0 1 6 7 9 12 14 11
sep = [0, 1000, 3000, 5000, 7000, 9000, 12000];
sepName = {'16:51', '17:31', '17:58', '18:30', '18:59', '19:42'};

if resultsToShow == 2
  sep = [0, 1000, 3000, 5000, 7000, 9000];
  sepName = {'17:27', '17:54', '18:27', '18:56', '19:35'};
  prefix = 'Consecutive';
  statNames = {'Loop/Distance_since_last_loc/', 'Distance_since_last_loc_under_50cm'};
endif

MapsN = length(sepName);

allCumResults = {};
allMaxResults = {};

for s=1:length(statNames)

avgResults = {};
maxResults = {};
totalResults = {};
absResults = {};

statName = strrep(statNames{s},'/','-');

for d=1:length(datasets)

if strcmp(statName,'Inliers_ratio_%')
  data = dlmread([dataDir '/' prefix num2str(datasets(d)) '-' 'Loop-Visual_inliers-' '.txt'], '\t', 1, 0, "emptyvalue", 0);
  dataWords = dlmread([dataDir '/' prefix num2str(datasets(d)) '-' 'Keypoint-Current_frame-words' '.txt'], '\t', 1, 0, "emptyvalue", 0);
  data(:, 2:end) = data(:, 2:end) ./ dataWords(:, 2:end) * 100;
elseif strcmp(statName, 'Odometry_average')
  data = dlmread([dataDir '/' prefix num2str(datasets(d)) '-' 'Memory-Distance_travelled-m' '.txt'], '\t', 1, 0, "emptyvalue", 0);
elseif strcmp(statName, 'Distance_since_last_loc_under_50cm')
  data = dlmread([dataDir '/' prefix num2str(datasets(d)) '-' 'Loop-Distance_since_last_loc-' '.txt'], '\t', 1, 0, "emptyvalue", 0);
else
  data = dlmread([dataDir '/' prefix num2str(datasets(d)) '-' statName '.txt'], '\t', 1, 0, "emptyvalue", 0);
endif
sessions = size(data,2)-1;

avgResultsTmp = zeros(sessions, length(sep)-1);
maxResultsTmp = zeros(sessions, length(sep)-1);
totalResultsTmp = zeros(sessions, length(sep)-1);
absResultsTmp = zeros(sessions, length(sep)-1);

for i = 1:sessions
  for j = 1:length(sep)-1
    x = data(:,1);
    y = data(:,i+1);
    y = y(x>=sep(j) & x<=sep(j+1), :);
    x = x(x>=sep(j) & x<=sep(j+1), :);
    if strcmp(statName, 'Odometry_average')
      y(2:end) = y(2:end) - y(1:end-1);
      y(y < 0.05) = 0;
    elseif strcmp(statName, 'Loop-Map_id-')
      y = y+1;
      y(y>0) = 1;
    end
    if strcmp(statName, 'Memory-RAM_estimated-MB') && RAMaddOverhead == 1
      % Valgrind estimated around 90 MB constant overhead
      y = y + 90;
      if datasets(d) == 7
        %% 135 MB overhead for BRISK kernel
        y = y + 135;
      elseif datasets(d) == 11 || datasets(d) == 111
        %% 645 MB (library cuda) + 800 MB (network) for SuperPoint
        y = y + 645+800;
      elseif datasets(d) == 13 || datasets(d) == 14
        %% 64 MB overhead for DAISY
        y = y + 64;
      endif
    endif
    nonzeros = y(y>0);
    if strcmp(statName, 'Loop-Map_id-')
      nonzeros = y;
    end
    if strcmp(statName, 'Distance_since_last_loc_under_50cm')
      y(y>0.55) = 0;
      y(isnan(y)) = 0;
      y(y>0) = 1;
      nonzeros = y;
    end
    if length(nonzeros) > 0
      avgValue = sum(nonzeros)/length(nonzeros);
      avgResultsTmp(i,j) = avgValue;
      maxResultsTmp(i,j) = max(nonzeros);
      totalResultsTmp(i,j) = length(nonzeros);
      absResultsTmp(i,j) = sum(nonzeros);
    endif
  endfor
endfor
avgResults{1,d} = avgResultsTmp;
maxResults{1,d} = maxResultsTmp;
totalResults{1,d} = totalResultsTmp;
absResults{1,d} = absResultsTmp;
endfor

% compute cumulative results
cumResults = zeros(sessions+2, length(datasets)+1);
for d=1:length(datasets)
  cumResults(1,d+1) = datasets(d);
  if sum(totalResults{1,d}, 2)
    cumResults(2:end-1,d+1) = sum(absResults{1,d}, 2) ./ sum(totalResults{1,d}, 2);
  endif
  cumResults(end,d+1) = sum(sum(absResults{1,d}(1:MapsN,1:MapsN).*eye(MapsN,MapsN))) / sum(sum(totalResults{1,d}(1:MapsN,1:MapsN).*eye(MapsN,MapsN)));
end
cumResults(2:end-1,1) = 1:sessions;

allCumResults{1,s} = statNames{s};
if strcmp(statNames{s}, 'Loop/Odom_correction_norm/m')
  cumResults(2:end,2:end) = cumResults(2:end,2:end) * 1000;
  allCumResults{1,s} = 'Loop/Odom_correction_norm/mm';
elseif strcmp(statNames{s}, 'Loop/Map_id/')
  cumResults(2:end,2:end) = cumResults(2:end,2:end) * 100;
endif
allCumResults{2,s} = round(cumResults);

% compute max results
cumMaxResults = zeros(sessions+2, length(datasets)+1);
for d=1:length(datasets)
  cumMaxResults(1,d+1) = datasets(d);
  if sum(totalResults{1,d}, 2)
    cumMaxResults(2:end-1,d+1) = max(maxResults{1,d}, [], 2);
  endif
  cumMaxResults(end,d+1) = max(max(maxResults{1,d}(1:MapsN,1:MapsN).*eye(MapsN,MapsN)));
end
cumMaxResults(2:end-1,1) = 1:sessions;

allMaxResults{1,s} = statNames{s};
allMaxResults{2,s} = cumMaxResults;

endfor % statNames


if resultsToShow == 2
  disp('30min') 
  for d=1:length(datasets)
    round(sum(absResults{1,d} .* eye(5,5)) / sum(totalResults{1,d} .*eye(5,5)) * 100)
  endfor
  disp('60min') 
  for d=1:length(datasets)
    round(sum(absResults{1,d} .* [[0;0;0;0] eye(4,4) ; [0 0 0 0 0]]) / sum(totalResults{1,d} .*[[0;0;0;0] eye(4,4) ; [0 0 0 0 0]]) * 100)
  endfor
  disp('120min') 
  for d=1:length(datasets)
    round(avgResults{1,d}(1,5) * 100)
  endfor
endif

