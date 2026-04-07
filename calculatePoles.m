function poles = calculatePoles(overshoot, settling_time)
%Compute the second order desired poles
%Based on the desired performance indexes of overshoot and settling time
%the function computes the desired second order closed-loop poles

    % Convert overshoot to damping ratio (zeta)
    zeta = -log(overshoot) / sqrt(pi^2 + log(overshoot)^2);
    
    % Calculate natural frequency (omega_n) using settling time
    w_n = 4 / (zeta * settling_time);
    
    % Calculate the poles of the second-order system
    sigma = -zeta * w_n;
    imag_part = w_n * sqrt(1 - zeta^2);
    
    % Return the poles as a complex number pair
    poles = [sigma + 1i * imag_part, sigma - 1i * imag_part];
end
