# pvfopid
[![View Padula-Visioli tuning algorithm for a FOPID controller on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/73396-padula-visioli-tuning-algorithm-for-a-fopid-controller)
Padula-Visioli tuning algorithm for a FOPID controller in CLS system with a first order plus dead time plant (FOPDT)

Algorithm was proposed in:

[1] F. Padula and A. Visioli, "Tuning rules for optimal PID and fractional-order PID controllers,"
    J. Process Control, vol. 21, no. 1, pp. 69–81, Jan. 2011.

Requirements:
- FOTF toolbox

Parameters:
- K - FOPDT gain
- T - FOPDT time constant
- L - FOPDT dead time

Output:
- SP14 - 5-element vector of coefficients [K_P, K_I, K_D, \mu, \nu, N] for the Set point task, M_1=1.4
- SP20 - coefficients for the Set point task, M_2=2.0
- LD14 - coefficients for the Load disturbance rejection task, M_1=1.4
- LD20 - coefficients for the Load disturbance rejection task, M_2=2.0
- GconSP14 - fractional-order transfer function (FOTF) of the FOPID controller based on SP14 coefficients
- GconSP20 - FOTF of the FOPID based on SP20 coefficients
- GconLD14 - FOTF of the FOPID based on LD14 coefficients
- GconLD20 - FOTF of the FOPID based on LD20 coefficients

Implemented by: 

Mariusz Matusiak <mmatusiak@iis.p.lodz.pl>

Lodz University of Technology, Poland
