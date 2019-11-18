%% Padula-Visioli tuning algorithm for a FOPID controller in CLS system with a first order plus dead time plant (FOPDT)
%  Algorithm was proposed in:
%  [1] F. Padula and A. Visioli, "Tuning rules for optimal PID and fractional-order PID controllers,"
%      J. Process Control, vol. 21, no. 1, pp. 69–81, Jan. 2011.
%  Requirements:
%  * FOTF toolbox
%  Parameters:
%  * K - FOPDT gain
%  * T - FOPDT time constant
%  * L - FOPDT dead time
%  Output:
%  * SP14 - 5-element vector of coefficients $[K_P, K_I, K_D, \mu, \nu, N]$ for the Set point task, $M_1=1.4$
%  * SP20 - coefficients for the Set point task, $M_2=2.0$
%  * LD14 - coefficients for the Load disturbance rejection task, $M_1=1.4$
%  * LD20 - coefficients for the Load disturbance rejection task, $M_2=2.0$
%  * GconSP14 - fractional-order transfer function (FOTF) of the FOPID controller based on SP14 coefficients
%  * GconSP20 - FOTF of the FOPID based on SP20 coefficients
%  * GconLD14 - FOTF of the FOPID based on LD14 coefficients
%  * GconLD20 - FOTF of the FOPID based on LD20 coefficients
%  Implemented by: Mariusz Matusiak <mmatusiak@iis.p.lodz.pl>
%  Lodz University of Technology, Poland
%%
function [SP14, SP20, LD14, LD20, GconSP14, GconSP20, GconLD14, GconLD20] = pv_controller(K, T, L)
if nargin ~= 3 || K <= 0 || T <= 0 || L < 0
    error('Not enough or invalid input arguments. Please provide positive K, T and L parameters of a FOPDT plant')
end
tau = L / (L + T);
if tau < 0.05
    disp('You can neglect the dead time')
elseif tau > 0.8
    disp('Consider dead time compensator')
end
SP14 = zeros([5 1]); %M1_1.4
SP20 = zeros([5 1]); %M2_2.0
LD14 = zeros([5 1]); %M1_1.4
LD20 = zeros([5 1]); %M2_2.0

% Set point M1=1.4
mu = 1;
if tau < 0.1
    nu = 1.1;
else
    nu = 1.2;
end
N = 10*T^(nu - 1);
a =  0.6503;
b = -0.9166;
c = -0.6741;
Kp = get_kp(tau, a, b, c, K);
a =  0.04701;
b = -0.2611;
c =  0.9276;
Ki = get_ki_kd(T, L, a, b, c, mu);
a =  0.3563;
b =  1.2000;
c =  0.0003108;
Kd = get_ki_kd(T, L, a, b, c, nu);
SP14(1) = Kp; SP14(2) = Ki; SP14(3) = Kd; SP14(4) = mu; SP14(5) = nu;
GconSP14 = get_gcon(Kp, Ki, Kd, mu, nu, N);

% Set point M2=2.0
mu = 1;
if tau < 0.1
    nu = 1.0;
elseif tau < 0.4
    nu = 1.1;
else
    nu = 1.2;
end
N = 10*T^(nu - 1);
a =  0.9294;
b = -0.9330;
c = -0.9205;
Kp = get_kp(tau, a, b, c, K);
a = -0.001427;
b = -1.0030;
c =  1.0310;
Ki = get_ki_kd(T, L, a, b, c, mu);
a =  0.4203;
b =  1.2290;
c =  0.01822;
Kd = get_ki_kd(T, L, a, b, c, nu);
SP20(1) = Kp; SP20(2) = Ki; SP20(3) = Kd; SP20(4) = mu; SP20(5) = nu;
GconSP20 = get_gcon(Kp, Ki, Kd, mu, nu, N);

% Load disturbance rejection M1=1.4
mu = 1;
if tau < 0.1
    nu = 1.0;
elseif tau < 0.4
    nu = 1.1;
else
    nu = 1.2;
end
N = 10*T^(nu - 1);
a =  0.2776;
b = -1.0950;
c = -0.1426;
Kp = get_kp(tau, a, b, c, K);
a =  0.6241;
b =  0.5573;
c =  0.0442;
Ki = get_ki_kd(T, L, a, b, c, mu);
a =  0.4793;
b =  0.7469;
c = -0.02393;
Kd = get_ki_kd(T, L, a, b, c, nu);
LD14(1) = Kp; LD14(2) = Ki; LD14(3) = Kd; LD14(4) = mu; LD14(5) = nu;
GconLD14 = get_gcon(Kp, Ki, Kd, mu, nu, N);

% Load disturbance rejection M2=2.0
mu = 1;
if tau < 0.1
    nu = 1.0;
elseif tau < 0.4
    nu = 1.1;
else
    nu = 1.2;
end
N = 10*T^(nu - 1);
a =  0.1804;
b = -1.4490;
c =  0.2319;
Kp = get_kp(tau, a, b, c, K);
a =  0.6426;
b =  0.8069;
c =  0.05627;
Ki = get_ki_kd(T, L, a, b, c, mu);
a =  0.5970;
b =  0.5568;
c = -0.09536;
Kd = get_ki_kd(T, L, a, b, c, nu);
LD20(1) = Kp; LD20(2) = Ki; LD20(3) = Kd; LD20(4) = mu; LD20(5) = nu;
GconLD20 = get_gcon(Kp, Ki, Kd, mu, nu, N);
end

% Internal helper methods
function [Kp] = get_kp(tau, a, b, c, K)
    if K ~= 0 
        Kp = 1/K * (a * (tau^b) + c);
    else
        error('K == 0!')
    end
end

function [KiKd] = get_ki_kd(T, L, a, b, c, mu_nu)
    if T ~= 0
        KiKd = T^mu_nu * (a * (L/T)^b + c);
    else
        error('T == 0!')
    end
end

function [Gcon] = get_gcon(Kp, Ki, Kd, mu, nu, N)
    if N ~= 0 && Ki ~= 0
        s = fotf('s');
        Gcon = Kp * ((Ki * s^mu + 1)/(Ki * s^mu)) * ((Kd * s^nu + 1)/(Kd/N * s^nu + 1));
    else
        error('N == 0 || Ki == 0!')
    end
end