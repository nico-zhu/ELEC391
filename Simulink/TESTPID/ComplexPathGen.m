%Script to Generate Path
%all measurements are in Centimeters
timeNeeded = 1.0;   %this will dictate the speed of the path.
timeSteps = timeNeeded ./ SimStepTime;

InputX = [45 30 28 24 31 25]';
InputY = [0 16 25 22 30 7]';

numlines = length(InputX)-1;
slope = zeros((numlines), 1);
const = slope;
Xstep = slope;
%Calculate slopes and constants and steps
%straight line of the from y = mx + c
for i = 1 : (numlines)
    slope(i) = (InputY(i+1) - InputY(i))/(InputX(i+1) - InputX(i));
    const(i) = InputY(i) - slope(i) * InputX(i);
    Xstep(i) = (InputX(i+1) - InputX(i))/timeSteps;
end

%Variables to store the position data
Xpos = zeros(length(SimTimeVector), 1);
Ypos = Xpos;
%initialize them
Xpos(1) = InputX(1);
Ypos(1) = InputY(1);

currentpos = 2;

%Generate the path
%generate lines sequentially
for j = 1: (numlines)
    for i = currentpos: length(SimTimeVector)
        if(SimTimeVector(i) <= (timeNeeded * j))
            Xpos(i) = Xpos(i - 1) + Xstep(j);
            Ypos(i) = Xpos(i) * slope(j) + const(j);
        elseif(j == numlines)
            Xpos(i) = Xpos(i - 1);
            Ypos(i) = Ypos(i - 1);
        else
            currentpos = i;
            break;
        end
    end
end

%finally Convert output to suitable datatype ~~Timeseries
XposT = timeseries(Xpos, SimTimeVector);
YposT = timeseries(Ypos, SimTimeVector);
figure(2)
plot(Xpos, Ypos)