function [time, value, first_time_navdata] = log2signals(input_filename, tag, signal, first_time_navdata, varargin)

calculate_first_time_navdata = false;
use_first_time_navdata = true;
% disp(nargin);
if ((nargin>=4) && (first_time_navdata < 0))
    first_time_navdata = 0;
    calculate_first_time_navdata = true;
    use_first_time_navdata = false;
end

if strcmp(class(input_filename), 'cell') ~= 0,
    m = input_filename;
else
    if strcmp(class(input_filename), 'char') ~= 0,
        display(['Loading file...']);
        m = txt2cellm(input_filename, [' ' ':']);
        display('File loaded');
    else 
        error('Wrong argument type');
    end
end

if size(varargin, 1) >= 1 && varargin{1} ~= 0,
    % Look for command range: first command MOVE and first command HOVER or
    % LAND
    first_test_line = -1;
    for i = 1:size(m, 2),
        if strcmp(m{i}(2), '[cmd]') ~= 0 && strcmp(m{i}(4), 'MOVE') ~= 0,
            % First MOVE
            first_test_line = i;
            break;
        end
    end
    if first_test_line < 0, error('Cannot find first MOVE command'); end;
    % Search HOVER or LAND after MOVE    
    last_test_line = -1;
    for i = first_test_line:size(m, 2),
        if strcmp(m{i}(2), '[cmd]') ~= 0 && (strcmp(m{i}(4), 'HOVER') ~= 0 || strcmp(m{i}(4), 'LAND') ~= 0),
            % Test stops
            last_test_line = i;
            break;
        end
    end
    if last_test_line < 0, error('Cannot find test end'); end;
else
    first_test_line = 1;
    last_test_line = size(m, 2);
end;

signal_index = -1;
output_index = 1;
time = [];
value = [];
tag = ['[' tag ']'];
first_time = -1;
for i = first_test_line:last_test_line,
    if strcmp(m{i}(2), tag) ~=0,
        if first_time < 0, 
            first_time = str2num(m{i}{1});
%             disp( ['first_time = ', num2str(first_time)] );
        end;
        if use_first_time_navdata
            time(output_index) = (str2num(m{i}{1}) - first_time_navdata) / 1e6;
        else
            time(output_index) = (str2num(m{i}{1}) - first_time) / 1e6;
        end
        
        if signal_index < 0,
            % Look for signal index
            for j = 3:2:size(m{i}, 2),
                if strcmp(m{i}(j), signal) ~= 0,
                    signal_index = j + 1;
                    break;
                end;
            end
        end
        if signal_index < 0, error(['Cannot find signal ' signal]); end;
        
        if size(varargin, 2) >= 2 && varargin{2} ~= 0,
            value{output_index} = m{i}{signal_index};
        else
%             disp(i);
%             disp( output_index );
%             disp( m{i}{signal_index} );
%             disp( str2num(strrep(m{i}{signal_index}, ',', '.')) );
            value(output_index) = str2num(strrep(m{i}{signal_index}, ',', '.'));
        end;
        output_index = output_index + 1;
    end
end

if size(varargin, 2) >= 3,
    desired_t = varargin{3};
    v_corr = [];
    v_index = 1;
    for i = 1:size(desired_t, 2),
        while v_index < size(time, 2) && desired_t(i) >= time(v_index + 1),
            v_index = v_index + 1;
        end;
        v_corr(i) = value(v_index);
    end
    value = v_corr;
    time = desired_t;
end

% disp( ['first_time = ', num2str(first_time)] );
% disp( ['calculate_first_time_navdata = ', num2str(calculate_first_time_navdata)] );
if calculate_first_time_navdata
    first_time_navdata = first_time;
end

end