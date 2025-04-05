clc;
warning('off','all');

% Parámetros de muestreo y normalización
ts = 0.1;                  % Tiempo de muestreo (segundos)
t = 0:ts:10;               % Vector de tiempo
maxValue = 1023;
minValue = 0;

% Preasignar vectores para datos
value1 = zeros(1, length(t));
value2 = zeros(1, length(t));
valueN1 = zeros(1, length(t));
valueN2 = zeros(1, length(t));

% Conexión a Arduino mediante el objeto serialport
% Asegúrate de que el puerto y baudrate sean correctos (ejemplo: 9600 bps)
s = serialport('COM4', 9600);
configureTerminator(s, "LF");  % Configura el terminador de línea si es necesario
pause(4);                      % Pausa para establecer la comunicación
disp('Recolectando datos...');

for k = 1:length(t)
    tic;
    flush(s);                  % Vacía el búfer
    data = readline(s);        % Lee una línea completa de datos
    
    % Se espera que los datos tengan el formato: <dato1,dato2>
    idx1 = strfind(data, '<');
    idx2 = strfind(data, '>');
    
    if ~isempty(idx1) && ~isempty(idx2) && idx2 > idx1
        % Extrae la cadena entre los caracteres '<' y '>'
        subStr = data(idx1+1 : idx2-1);
        % Separa los datos usando la coma como separador
        Data = strsplit(subStr, ',');
        
        if numel(Data) >= 2
            value1(k) = str2double(Data{1});
            value2(k) = str2double(Data{2});
        else
            if k > 1
                value1(k) = value1(k-1);
                value2(k) = value2(k-1);
            end
        end
    else
        % Si no se reciben datos válidos, se mantiene el valor anterior
        if k > 1
            value1(k) = value1(k-1);
            value2(k) = value2(k-1);
        end
    end
    
    % Normalización de los valores (0 a 1)
    valueN1(k) = (value1(k) - minValue) / (maxValue - minValue);
    valueN2(k) = (value2(k) - minValue) / (maxValue - minValue);
    

end

disp('Datos recolectados...');

% Cierra la conexión serial
delete(s);
clear s;

% Graficar los resultados
figure;
subplot(221)
plot(t, value1);
title('Ángulo real de pitch');
xlabel('Tiempo (s)'); ylabel('Valor');

subplot(222)
plot(t, valueN1);
title('Ángulo pitch normalizado');
xlabel('Tiempo (s)'); ylabel('Valor');

subplot(223)
plot(t, value2);
title('Ángulo real de roll');
xlabel('Tiempo (s)'); ylabel('Valor');

subplot(224)
plot(t, valueN2);
title('Ángulo roll normalizado');
xlabel('Tiempo (s)'); ylabel('Valor');
