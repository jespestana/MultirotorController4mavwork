function output = txt2cellmv2(file, seps)

%It converts a space separated field text file into a string matrix
f = fopen(file, 'r');
output = {};
nseps = max(size(seps));
while 1
    tline = fgetl(f);
    if ~ischar(tline), break, end
    if (nseps > 1),
        for i=2:nseps,
            tline = strrep(tline, seps(i), seps(1));
        end;
    end;
%     words = split(tline, seps(1));
    tline = regexprep(tline, [seps(1) '*'], seps(1));
    words = regexp(tline, seps(1), 'split');
    output{length(output) + 1} = words;
end
fclose(f);
end