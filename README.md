# Проект дифференциального двухколесного бота

Параметры бота задаются в файле config.json. Стартовое положение и ориентация: <img src="https://latex.codecogs.com/gif.latex?(x,&space;y,&space;\theta)&space;=&space;(0,&space;0,&space;0)" title="(x, y, \theta) = (0, 0, 0)" />.

### Управление ботом через HTTP API:
 * POST goTo - отправить бот к цели. Параметры в json: x,y - желаемое положение бота, theta - желаемая ориентация бота, v - максимальная    скорость, а - ускорение;
 * GET stop - остановить бот и сбросить цель;
 * GET pause - остановить бот без сброса цели;
 * GET resume - возобновить движение к цели;
 * GET status - возвращает текущее положение,ориентацию и скорость бота в json.
 
Управление ботом осуществляется заданием скорости вращения каждого колеса - <img src="https://latex.codecogs.com/gif.latex?V_r,&space;V_l" title="V_r, V_l" />. Положение и ориентация бота рассчитываются на основании пройденного пути каждого колеса. В данной модели отсутствуют энкодеры, будем считать, что мы получаем угол поворота каждого колеса в радианах.

#### Расчет пройденного расстояния:
<br><img src="https://latex.codecogs.com/gif.latex?D_r&space;=&space;R&space;*&space;\Delta_r" title="D_r = R * \Delta_r" /><br>
<img src="https://latex.codecogs.com/gif.latex?D_l&space;=&space;R&space;*&space;\Delta_l" title="D_l = R * \Delta_l" /><br>
<img src="https://latex.codecogs.com/gif.latex?D_c&space;=&space;\frac{D_r&space;&plus;&space;D_l}{2}" title="D_c = \frac{D_r + D_l}{2}" /><br>

#### Вычисление нового положения и ориентации:
<br><img src="https://latex.codecogs.com/gif.latex?x_{t&plus;1}&space;=&space;x_t&space;&plus;&space;D_c&space;*&space;cos(\theta_t)" title="x_{t+1} = x_t + D_c * cos(\theta_t)" /><br>
<img src="https://latex.codecogs.com/gif.latex?y_{t&plus;1}&space;=&space;y_t&space;&plus;&space;D_c&space;*&space;sin(\theta_t)" title="y_{t+1} = y_t + D_c * sin(\theta_t)" /><br>
<img src="https://latex.codecogs.com/gif.latex?\theta_{t&plus;1}&space;=&space;\theta_t&space;&plus;&space;\frac{D_r&space;-&space;D_l}{L}" title="\theta_{t+1} = \theta_t + \frac{D_r - D_l}{L}" /><br>

#### Расчет заданий скорости каждого колеса
<br><img src="https://latex.codecogs.com/gif.latex?V_r&space;=&space;\frac{2*V&space;-&space;\omega*L}{2R}" title="V_r = \frac{2*V - \omega*L}{2R}" /><br>
<img src="https://latex.codecogs.com/gif.latex?V_l&space;=&space;\frac{2*V&space;&plus;&space;\omega*L}{2R}" title="V_l = \frac{2*V + \omega*L}{2R}" />
