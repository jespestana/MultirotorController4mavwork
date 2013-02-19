function R = rotk(k, theta)
% Devuelve la matriz de rotación de un ángulo theta sobre un vector k
ct = cos(theta); st = sin(theta);
vt = 1 - ct;
kx = k(1); ky = k(2); kz = k(3);
% Fórmula de Rodrigues
R = [   kx^2*vt+ct      kx*ky*vt-kz*st      kx*kz*vt+ky*st;
        kx*ky*vt+kz*st  ky^2*vt+ct          ky*kz*vt-kx*st;
        kx*kz*vt-ky*st  ky*kz*vt+kx*st      kz^2*vt+ct      ];
