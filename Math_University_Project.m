%% Modelo Trabajo Individual

% Este modelo tiene 4 secciones: la introducción mas los 3 problemas.
% Escribe tu codigo para cada problema en la sección que corresponde. Por
% favor, no añada mas secciones.

% alpha = 15/31;
% beta = 1/12;

%% Problema 1

clc
clear all

syms f(x);

alpha = 15/31;
beta = 1/12;

f(x) = (1/(x+alpha)) + beta*x;
fp = diff(f);
fpp = diff(fp);

F = matlabFunction (f(x));
Fp = matlabFunction(fp);
Fpp = matlabFunction (fpp);

numIts = 50;
X = zeros(1,numIts+1);
X(1) = 1;
gamma = 0.25;

for i = 1:numIts
    X(i+1) = X(i) - gamma*fp(X(i));
end

xx = linspace(-2,2,10); 
plot(xx,F(xx))

hold on
plot(xx,Fp(xx))
plot(xx,0*xx,'k')
hold off

fzero(Fp,0.9)

%% Problema 2

clc
clear all

alpha = 15/31;
beta = 1/12;

syms f(x,y) t s;

f(x,y) = 1 - 10 * ((alpha * t.^2) + (beta * s.^2));
Intt = int(f,t,-y,y);
Ints = int(Intt,s,-x,x);

f(x,y) = Ints;
fp(x,y) = diff(f);
fpp = diff(fp);

F = matlabFunction(f);
Fp = matlabFunction(fp);
Fpp = matlabFunction(fpp);

gradI = gradient(f);
GradI = matlabFunction(gradI);

numIts = 50;
gamma = 0.1;
V = zeros(1,numIts+1);
W = zeros(1,numIts+1);
V(1) = 0.1;
W(1) = 0.1;
for i = 1:numIts
    grad_actual = GradI(V(i),W(i));
    V(i+1) = V(i) + gamma*grad_actual(1);
    W(i+1) = W(i) + gamma*grad_actual(2);
    norm(grad_actual)
end

xx = linspace(0, 1, 100);
yy = linspace(0, 1, 100);

[XX, YY] = meshgrid(xx, yy);
snapnow, surf(XX,YY, F(XX,YY))

xlabel('eje x');
ylabel('eje y');
zlabel('eje z');

x_max = V(end)
y_max = W(end)

fp_max = Fp(x_max, y_max)
Fpp_max = Fpp(x_max, y_max)

%% Problema 3

clc
clear all

alpha = 15/31;
beta = 1/12;

rng(1);
n = 201;
x = linspace(0,2,n)';
y = alpha + beta*x + sin(2*pi*x) + 0.1*randn(n,1);
g1 = @(x) 0*x + 1;
g2 = @(x) x;
g3 = @(x) sin(2*pi*x);

V = [g1(x) g2(x) g3(x)];
[Q,R] = qr(V);
a = R\(Q'*y);

f = @(x) a(1)*g1(x) + a(2)*g2(x) + a(3)*g3(x);

plot(x,y,'o',x,f(x),'-')
legend('Datos','Ajuste')
