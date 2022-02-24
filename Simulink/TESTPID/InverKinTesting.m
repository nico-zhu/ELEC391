clear all

testX = [ 10 -8 10 -25]';
testY = [ 32 -8 -14 9]';

t1 = [ 0 0 0 0]';
t2 = [ 0 0 0 0]';

for i = 1: 4
    [t1(i), t2(i)] = InverseKin(testX(i), testY(i));
end


Larm1 = 20;
Larm2 = 15;

figure(1)
hold on
plot(testX, testY)

for i = 1 : 4
    arm1X = Larm1 * cos(t1(i)*pi/180);
    arm1Y = Larm1 * sin(t1(i)*pi/180);
    arm1Xx = [0 arm1X];
    arm1Yy = [0 arm1Y];

    arm2X = Larm2 * cos(t2(i)*pi/180);
    arm2Y = Larm2 * sin(t2(i)*pi/180);
    arm2Xx = [arm1X, arm2X + arm1X];
    arm2Yy = [arm1Y, arm2Y + arm1Y];
    
    plot(arm1Xx, arm1Yy, '-r');
    plot(arm2Xx, arm2Yy, '-r');
end

ylim([-40 40]);
xlim([-40 40]);


