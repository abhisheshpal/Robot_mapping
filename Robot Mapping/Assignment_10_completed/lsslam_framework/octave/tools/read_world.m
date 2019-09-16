function landmarks = read_world(filename)
    % Reads the world definition and returns a structure of landmarks.
    %
    % filename: path of the file to load
    % landmarks: structure containing the parsed information
    %
    % Each landmark contains the following information:
    % - id : id of the landmark
    % - x  : x-coordinate
    % - y  : y-coordinate
    %
    % Examples:
    % - Obtain x-coordinate of the 5-th landmark
    %   landmarks(5).x
    input = fopen(filename);
    landmarks= struct('X', cell(size(input)),'edges' ,[],'idLookup' , []);

    while(~feof(input))
        line = fgetl(input);
        data = strsplit(line, ' ');

        landmarks = struct('X', str2double(data{1}),'edges',[],'idLookup' , []);
        %landmarks(end+1) = landmark;
    end

    landmarks = landmarks(2:end);

    fclose(input);
    
end
