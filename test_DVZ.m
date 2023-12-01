ax = 10;
by = 5;
num = 1000;
alpha = linspace(-pi, pi, num);
dh = ((ax * cos(alpha)).^2 + (by * sin(alpha)).^2).^0.5;

numStep = num/10;
beam.x = zeros(num, numStep);
beam.x(:, numStep) = (dh.*cos(alpha)).';
beam.x(:, 2:numStep-1) = ((dh/(numStep-1)).*cos(alpha)).' .* (1:numStep-2);