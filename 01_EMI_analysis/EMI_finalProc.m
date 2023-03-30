% To measure the time required to run
tic

warning('off');     % To supress "Warning: Table variable names were modified to make them valid MATLAB identifiers. The original names are saved in the VariableDescriptions property."
                    % getted from "readtable" function

all_rezData = [];            % the final data

file_finalResults = 'EMI_results_all_17.txt';

for ii = 0 : 7              % Aii_jj.csv   ii => specific system   jj => no. of recorfing [01, 02, 03]
   for jj = 1 : 3
        file = ['A' sprintf('%02d_%02d', ii, jj) '.csv'];
       
        my_data = readtable (file);

        % get table dimensions
        [linii, coloane] = size (my_data);

        % first I must search after:

        % sensor_gps_jamming_indicator (504)
        % sensor_gps_noise_per_ms      (508)
        % sensor_gps_satelites_used    (510)

        % vehicle_gps_position_jamming_indicator (AAM)  (715)
        % vehicle_gps_position_noise_per_ms      (AAQ)  (719) 
        % vehicle_gps_position_satelites_used           (721)  

        % find table column number by column name
        sGPS_jIn = find(string(my_data.Properties.VariableNames) == "sensor_gps_jamming_indicator");
        sGPS_nMs = find(string(my_data.Properties.VariableNames) == "sensor_gps_noise_per_ms");
        sGPS_sUs = find(string(my_data.Properties.VariableNames) == "sensor_gps_satellites_used");

        vGPS_jIn = find(string(my_data.Properties.VariableNames) == "vehicle_gps_position_jamming_indicator");
        vGPS_nMs = find(string(my_data.Properties.VariableNames) == "vehicle_gps_position_noise_per_ms");
        vGPS_sUs = find(string(my_data.Properties.VariableNames) == "vehicle_gps_position_satellites_used");

        % for sensor_gps_jamming_indicator
        %===========================
        % 1. extract the coresponding column
        C_sGPS_jIn = my_data(:, sGPS_jIn);
        % 2. finding which rows in table contain NaN
        nanIdx = isnan(C_sGPS_jIn{:,1});
        % 3. complement nanIdx
        not_nanIdx = ~any(nanIdx, 2);
        % 4. extract only the value
        C_sGPS_jIn = C_sGPS_jIn(not_nanIdx,:);
        % 5. convert to array
        C = table2array (C_sGPS_jIn);
        % 6. do the math
        mean_sGPS_jIn = mean (C);
        stdv_sGPS_jIn = std (C);

        no_samplesSensor = size (C);

        % for sensor_gps_noise_per_ms
        %===========================
        % 1. extract the coresponding column
        C_sGPS_nMs = my_data(:, sGPS_nMs);
        % 2. finding which rows in table contain NaN
        nanIdx = isnan(C_sGPS_nMs{:,1});
        % 3. complement nanIdx
        not_nanIdx = ~any(nanIdx, 2);
        % 4. extract only the value
        C_sGPS_nMs = C_sGPS_nMs(not_nanIdx,:);
        % 5. convert to array
        C = table2array (C_sGPS_nMs);
        % 6. do the math
        mean_sGPS_nMs = mean (C);
        stdv_sGPS_nMs = std (C);
        
        % for sensor_gps_satelites_used
        %===========================
        % 1. extract the coresponding column
        C_sGPS_sUs = my_data(:, sGPS_sUs);
        % 2. finding which rows in table contain NaN
        nanIdx = isnan(C_sGPS_sUs{:,1});
        % 3. complement nanIdx
        not_nanIdx = ~any(nanIdx, 2);
        % 4. extract only the value
        C_sGPS_sUs = C_sGPS_sUs(not_nanIdx,:);
        % 5. convert to array
        C = table2array (C_sGPS_sUs);
        % 6. do the math
        mean_sGPS_sUs = mean (C);
        stdv_sGPS_sUs = std (C);
        min_sGPS_sUs  = min (C);
        max_sGPS_sUs  = max (C);
        
        local_sensorsData = [no_samplesSensor(1) mean_sGPS_jIn stdv_sGPS_jIn mean_sGPS_nMs stdv_sGPS_nMs mean_sGPS_sUs stdv_sGPS_sUs min_sGPS_sUs max_sGPS_sUs];
        
        %==========================================================================

        % vehicle_gps_position_jamming_indicator
        %===========================
        % 1. extract the coresponding column
        C_vGPS_jIn = my_data(:, vGPS_jIn);
        % 2. finding which rows in table contain NaN
        nanIdx = isnan(C_vGPS_jIn{:,1});
        % 3. complement nanIdx
        not_nanIdx = ~any(nanIdx, 2);
        % 4. extract only the value
        C_vGPS_jIn = C_vGPS_jIn(not_nanIdx,:);
        % 5. convert to array
        C = table2array (C_vGPS_jIn);
        % 6. do the math
        mean_vGPS_jIn = mean (C);
        stdv_vGPS_jIn = std (C);

        no_samplesVehicle = size (C);

        % for vehicle_gps_position_noise_per_ms
        %===========================
        % 1. extract the coresponding column
        C_vGPS_nMs = my_data(:, vGPS_nMs);
        % 2. finding which rows in table contain NaN
        nanIdx = isnan(C_vGPS_nMs{:,1});
        % 3. complement nanIdx
        not_nanIdx = ~any(nanIdx, 2);
        % 4. extract only the value
        C_vGPS_nMs = C_vGPS_nMs(not_nanIdx,:);
        % 5. convert to array
        C = table2array (C_vGPS_nMs);
        % 6. do the math
        mean_vGPS_nMs = mean (C);
        stdv_vGPS_nMs = std (C);
        
        % for vehicle_gps_position_satelites_used
        %===========================
        % 1. extract the coresponding column
        C_vGPS_sUs = my_data(:, vGPS_sUs);
        % 2. finding which rows in table contain NaN
        nanIdx = isnan(C_vGPS_sUs{:,1});
        % 3. complement nanIdx
        not_nanIdx = ~any(nanIdx, 2);
        % 4. extract only the value
        C_vGPS_sUs = C_vGPS_sUs(not_nanIdx,:);
        % 5. convert to array
        C = table2array (C_vGPS_sUs);
        % 6. do the math
        mean_vGPS_sUs = mean (C);
        stdv_vGPS_sUs = std (C);
        min_vGPS_sUs  = min (C);
        max_vGPS_sUs  = max (C);   
        
        local_vehicleData = [no_samplesVehicle(1) mean_vGPS_jIn stdv_vGPS_jIn mean_vGPS_nMs stdv_vGPS_nMs mean_vGPS_sUs stdv_vGPS_sUs min_vGPS_sUs max_vGPS_sUs];
        
        all_rezData = [all_rezData; (ii + jj/100) local_sensorsData local_vehicleData];
        
        fprintf ('.');      % to get someting similar with a progress bar
   end
end

header = ['recordingInfo #samplSensor mean_sGPS_jamm stdv_sGPS_jamm mean_sGPS_nMs stdv_sGPS_nMs mean_sGPS_used stdv_sGPS_used min_sGPS_used max_sGPS_used #sampl_veh mean_vGPS_jamm stdv_vGPS_jamm mean_vGPS_nMs stdv_vGPS_nMs mean_vGPS_used stdv_vGPS_used min_vGPS_used max_vGPS_used'];

fid = fopen(file_finalResults,'wt');
fprintf(fid,'%s\n',header);
for lin = 1 : size(all_rezData,1)
   fprintf(fid, '%d ', all_rezData(lin,:));
   fprintf(fid, '\n');
end
fclose(fid);

message = 'EMI data extraction is finished !!!!!!'

clear

%  reads adn display the elapsed time
toc
