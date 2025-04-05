% Limpiar entorno de trabajo
clear; 
clc; 
close all;
in_max=180;
in_min=-180;

% Definir el número de muestras por cada par de valores
N_samples = 3000;
% Definir los cuatro valores de la secuencia
seq1 = repmat([1; 1], 1, N_samples);
seq2 = repmat([-1; -1], 1, N_samples);
seq3 = repmat([1; -1], 1, N_samples);
seq4 = repmat([-1; 1], 1, N_samples);
% Concatenar las secuencias para formar el vector final de 2x12000
T = [seq1, seq2, seq3, seq4];

P = rows2vars(readtable('datos_MPU.xlsx', 'Sheet', 'datos'));
P = P(1:2,2:12001);
P = table2array(P);
P = 2 * (P - in_min) ./ (in_max - in_min) - 1; % Normalización

save('DataSet.mat', 'P', 'T');

