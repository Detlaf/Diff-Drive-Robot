# Differential 2-wheel bot

Parameters are set in the config.json file. Starting position and orientation: <img src="https://latex.codecogs.com/gif.latex?(x,&space;y,&space;\theta)&space;=&space;(0,&space;0,&space;0)" title="(x, y, \theta) = (0, 0, 0)" />.

### Bot control via HTTP API:
 * POST goTo - send the bot to the destination. Json parameters: x,y - desired position, theta - desired orientation, v - maximum velocity, а - acceleration;
 * GET stop - stop the bot and reset destination;
 * GET pause - stop the bot without resetting the destination;
 * GET resume - resume moving to the destination;
 * GET status - returns a json with current position, orientation and velocity.
 
In order to control the bot it is necessary to specify the rotation velocity for each wheel - <img src="https://latex.codecogs.com/gif.latex?V_r,&space;V_l" title="V_r, V_l" />. Position and orientation of the bot are calculated using  distance covered by each wheel. This model does not have encoders, so we assume we receive data for each wheel.

#### Calculating covered distance (UpdateOdometry method):
<br><img src="https://latex.codecogs.com/gif.latex?D_r&space;=&space;R&space;*&space;\Delta_r" title="D_r = R * \Delta_r" /><br>
<img src="https://latex.codecogs.com/gif.latex?D_l&space;=&space;R&space;*&space;\Delta_l" title="D_l = R * \Delta_l" /><br>
<img src="https://latex.codecogs.com/gif.latex?D_c&space;=&space;\frac{D_r&space;&plus;&space;D_l}{2}" title="D_c = \frac{D_r + D_l}{2}" /><br>

#### Calculating new position and orientation (UpdateOdometry method):
<br><img src="https://latex.codecogs.com/gif.latex?x_{t&plus;1}&space;=&space;x_t&space;&plus;&space;D_c&space;*&space;cos(\theta_t)" title="x_{t+1} = x_t + D_c * cos(\theta_t)" /><br>
<img src="https://latex.codecogs.com/gif.latex?y_{t&plus;1}&space;=&space;y_t&space;&plus;&space;D_c&space;*&space;sin(\theta_t)" title="y_{t+1} = y_t + D_c * sin(\theta_t)" /><br>
<img src="https://latex.codecogs.com/gif.latex?\theta_{t&plus;1}&space;=&space;\theta_t&space;&plus;&space;\frac{D_r&space;-&space;D_l}{L}" title="\theta_{t+1} = \theta_t + \frac{D_r - D_l}{L}" /><br>

#### Calculating velocity for each wheel (UnicycleToDifferential method):
<br><img src="https://latex.codecogs.com/gif.latex?V_r&space;=&space;\frac{2*V&space;-&space;\omega*L}{2R}" title="V_r = \frac{2*V - \omega*L}{2R}" /><br>
<img src="https://latex.codecogs.com/gif.latex?V_l&space;=&space;\frac{2*V&space;&plus;&space;\omega*L}{2R}" title="V_l = \frac{2*V + \omega*L}{2R}" />

#### PID-controller (GoToGoal method)
Angle error serves as input for the controller, the calculation is performed using atan2 function in order to limit the value <img src="https://latex.codecogs.com/gif.latex?[-\pi,&space;\pi]" title="[-\pi, \pi]" />. 
For the integral part of the controller we calculate accumulated error, for the differential part - error delta: <br><br><img src="https://latex.codecogs.com/gif.latex?e&space;=&space;atan2(sin(\theta_d&space;-&space;\theta),&space;cos(\theta_d&space;-&space;\theta))" title="e = atan2(sin(\theta_d - \theta), cos(\theta_d - \theta))" /><br>
Angle velocity is the output:<br><br>
<img src="https://latex.codecogs.com/gif.latex?\omega&space;=&space;K_p*e&space;&plus;&space;K_i*\sum&space;e&space;&plus;&space;K_d*\Delta&space;e" title="\omega = K_p*e + K_i*\sum e + K_d*\Delta e" />

The destination is assumed to be reached, if bot's orientation and its distance from the goal are less than error value, specified in the config.json.
Formula for calculating distance: <br><img src="https://latex.codecogs.com/gif.latex?d&space;=&space;\sqrt{(x_g&space;-&space;x)^2&space;&plus;&space;(y_g&space;-&space;y)^2)}" title="d = \sqrt{(x_g - x)^2 + (y_g - y)^2)}" />

#### States
The bot has three possible states: MOVING, PAUSED, STOPPED.
* State is switched to MOVING when POST GoTo is received;
* GET pause switches state to PAUSED, velocity is set to 0;
* GET stop swithces state to STOPPED, velocity is set to 0, goal resets;
* As soon as the goal is reached the bot switches its state to STOPPED;
* The bot can switch its state to PAUSED only if it was in the MOVING state before.
