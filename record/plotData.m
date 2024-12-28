#!/usr/bin/octave
clear variables; close all
dataRec=load('datalog.log');
simTime=dataRec(:,1);
posDes_W=dataRec(:,2:4);
hipPos_W=dataRec(:,5:7);
basePos=dataRec(:,8:10);

ax(1) = subplot(3, 1, 1);
hold on;
plot(simTime, posDes_W(:, 1));
plot(simTime, posDes_W(:, 2));
plot(simTime, posDes_W(:, 3));
title('posDes_W');
legend('x', 'y', 'z');

ax(2) = subplot(3, 1, 2);
hold on;
plot(simTime, hipPos_W(:, 1));
plot(simTime, hipPos_W(:, 2));
plot(simTime, hipPos_W(:, 3));
title('hipPos_W');
legend('x', 'y', 'z');

ax(3) = subplot(3, 1, 3);
hold on;
plot(simTime, basePos(:, 1));
plot(simTime, basePos(:, 2));
plot(simTime, basePos(:, 3));
title('basePos');
legend('x', 'y', 'z');

linkaxes(ax, 'xy');

pause();

