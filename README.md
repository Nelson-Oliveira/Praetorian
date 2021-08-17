# Praetorian

ROS powered Autonomous Mobile Robot

<a href="https://robosavvy.com/store/dagu-wild-thumper-6wd-black-w-wheel-encoders-75-1-gearboxes.html">Wild Thumper 6WD</a> + <a href="https://robosavvy.com/store/dagu-t-39-rex-robot-motor-controller.html">T'REX Motor Controller</a> + <a href="https://robosavvy.com/store/spider-controller-with-atmega-2560.html">Spyder Controller</a> + <a href="https://robosavvy.com/store/raspberry-pi-3-b.html">Raspberry Pi3</a> Running ROS
<br>
<p>For the Platform I'm using the Wild Thumper 6WD (75:1) with Quadrature Encoders</p>
<p>For the Motor Controller I'm using Dagu T'REX motor controller (Flashed with TREX_controller2)</p>
<p>I'm Using an Arduino Spider Controller(Flashed with T_REX_I2C_praetorian) communicating via I2C with the T'REX, and Via Serial with the Raspberry Pi </p>

<p>The raspberry Pi is running ROS and acts as the "Brain" of the whole operation </p>


---------------------------------------------------------------------------------------------

<h3>*** V0.2.1 ***</h3>

<h3>Changelog</h4>

<p>-- Added Front and Back Sharp IR sensors</p>
<p>-- Added Front and Back HC-SR04 Sonar sensors</p>
<p>-- Added Turret</p>
<p>-- Added a "filter" to the Turret RC code not to be constantly updating the position</p>

<h3>Tests</h3>

<p><b>- Sharp IR sensors</b></p> 
<p>-- Both Sensors Tested:</p>
<p>--- Distance - Tested and working</p>
<p>--- Printing to serial - Tested and Working</p>
<p>--- ROS - Previously tested with one sensor, needs testing with both sensors - Probably Working</p>

<p><b>- HC-SR04 Sonar sensors</b></p> 
<p>-- Both Sensors Tested:</p>
<p>--- Distance - Tested and working but needs more testing</p>
<p>--- Printing to serial - Tested and Working</p>
<p>--- ROS - Not Tested</p>

<p><b>- Turret</b></p> 
<p>None</p>

<h3>Notes</h3>

<p><b>- HC-SR04</b></p> 
<p>-- Due to 30° angle it detects the floor at around 62cm</p>

<p><b>- Quadrature Encoders</b></p> 
<p>-- One side working, the other needs testing when the motor arrives</p>
<p>-- Encoder counts needs to be adjusted ( Measure meter by meter)</p>
<p>-- One of the sides might need to be changed to negative due to the motor being on the other side</p>

<p><b>- ROS dirving</b></p> 
<p>-- Not working - The problem might be:</p>
<p>--- I2C communication</p>
<p>--- PID</p>
<p>--- More to add</p>

<p><b>- Turret</b></p> 
<p>-- Added the RC still no ROS added</p>
<p>-- Added a "filter" to the RC code not to be constantly updating the position</p>
<p>-- The response time is too slow, need to see why probably too many delays in the loop</p>
