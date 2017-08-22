% Copyright © 2017 Andrew Smart
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or
% (at your option) any later version.
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% You should have received a copy of the GNU General Public License along
% with this program; if not, write to the Free Software Foundation, Inc.,
% 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
test = 1

# https://www.micromo.com/technical-library/dc-motor-tutorials/motor-calculations
Torque_Nm = .031090 #From impeller.m calculation
rpm = 5400
w_rad_per_min=rpm*2*pi;
w_rad_per_sec=w_rad_per_min/60;
P_rot = Torque_Nm * w_rad_per_sec;
P_rot_To_Watts_coeff = 0.1047;
P_rot_watts = P_rot * P_rot_To_Watts_coeff
P_loss_J_per_s = .2 * P_rot_watts; # assume 80% we're at efficiency on motor load curve
P_electrical_watts = P_rot_watts + P_loss_J_per_s
P_electrical_amps = P_electrical_watts / 12

# Document on LRK Motor Analysis, outer rotor, surface mounted magnets
# http://www.femm.info/examples/lrk40/lrk-bldc.pdf

#-----------------
# Preliminary Definitions
#-----------------
mu_0_H_per_m = 4 * pi * 10^-7
# The stored energy in a magnet, called magnet performance or
# maximum energy product (BH_max) is typically measured in units of
# megagauss-oersteds (MGOe). One MGOe is approximately equal to 7957.74715 J/m^3
MGOe = 10^6
A_rms = @(A) sqrt(2) * A;
V_rms = @(V) sqrt(2) * V;

#-----------------
# Machine Geometry
#-----------------
r_stator_mm = 12.8 #Outer radius of the stator
r_rotor_mm = 14.14 #Inner radius of the rotor (back-iron inner radius)
t_m_mm = 0.9 #Radial thickness of each magnet
w_m_mm = 5 #Width of each magnet
n = 15 #Number of turns per coil
p = 7 #Number of pole pairs on the rotor (14 magnets on the rotor = 7 pole pairs)
slots = 12 #Number of slots/teeth
g_tot_mm = r_rotor_mm - r_stator_mm
h_mm = 5 #Axial length of the stator stack
d_slot_mm = 0.77 #radial depth of slot teeth from the coil
w_slot_mm = 1.6 #width of the slot teeth gap in front of coil

#-----------------
# Magnet Properties
#-----------------
BH_max = 40#*MGOe #Energy product of the magnets
# Remanennce of the permanent magnet, assuming that the magnet's permeability is the same
# as free space, an OK assumption for NdFeB or SmCo magnets
B_r_Telsa = sqrt(4*mu_0_H_per_m*BH_max)
#B_r_Telsa = 1.264911T
if test == 1
	#assert(B_r_Telsa,1.264911,0.0001)
end

#-----------------
# Wire Properties
#-----------------
# Conductivity of the winding material,
# 58 MS/m is the conductivity of copper at room temperature
sigma_S_per_m = 58*10^6
AWG = 22 #Guage of wire used to wind the stator

#-----------------
# Torque estimation, double-check trapezoidal/teeth assumptions
#-----------------
i_phase_amps = 4.0 #Design current amplitude
# Where K_p represents the height of the plateau on a
# roughly trapezoidal back-EMF waveform for a phase.
K_p = 2 * (r_rotor_mm+r_stator_mm)/(r_rotor_mm-r_stator_mm)*B_r_Telsa*h_mm*t_m_mm*n
#K_p = 3.433101*10^-3 Wb
if test == 1
	assert(K_p,3.433101*10^-3,0.0001)
end
# Torque produced by the motor when driven with 2 phases
# on and one off with the two on phases carry currents of
# amplitude i_phase_amps
tau = 2 * K_p * i_phase_amps
#tau=0.027465 Nm
if test == 1
	assert(tau,0.027465,0.0001)
end

#-----------------
# Self-Inductance of Each Phase
#-----------------
# Magnetic reluctance of the air gap between the rotor and stator.
# This uses an old but good kludge of augmenting the area
# of the flux path by the air gap width times the
# perimeter of the gap
R_gap = (r_rotor_mm - r_stator_mm) / (mu_0_H_per_m*(pi*(r_rotor_mm+r_stator_mm))/slots*(h+2*(r_rotor_mm-r_stator_mm)))
R_leak = w_slot_mm/(mu_0_H_per_m*(d_slot_mm+2*w_slot_mm)*(h+2*w_slot_mm))
# Inductance due to flux that crosses from the stator
# to the rotor and back
L_gap = 2 * n^2 / R_gap
#L_gap = 0.022858 mH
if test == 1
	assert(L_gap,0.022858,0.0001)
end
# Leakage inductance from flux that crosses over
# to the neighboring unwound poles
L_leak = 4 * n^2 / R_leak
#L_leak = 0.023011 mH
if test == 1
	assert(L_leak,0.023011,0.0001)
end
L_phase = L_leak + L_gap #Inductance of each phase
#L_phase = 4.586942*10^-5 H
if test == 1
	assert(L_phase,4.586942*10^-5,0.0001)
end

#-----------------
# Resistance of Each Phase
#-----------------
# End turn diameter. We will guesstimate that this is
# the same as the stator tooth pitch at the surfarce
# of the stator
d_turn = 2*pi*r_stator_mm/slots
# Total length of wire in one phase, guesstimated to
# the circumference of a circle with the "end turn diameter"
# computed above, plus the length of the wire required
# to run down one side of the slot and back the other.
# The leading 2 is because there are two n-turn coils
# per phase.
I_wire = 2*n*(pi*d_turn+2*h)
# A convenient formula for wire diameter as a function
# of AWG wire guage, regressed from a published table
# of wire sizes
d_wire_inch = @(awg) (0.325105)*e^(-0.115958*awg); #anonymous function
a_wire = pi*d_wire_inch(AWG)^2/4
R_phase_ohms = I_wire/(sigma_S_per_m*a_wire)
#R_phase_ohms = 0.049301
if test == 1
	assert(R_phase_ohms,0.049301,0.0001)
end

#-----------------
# Equivalent Circuit, Operating Point
#-----------------
# Amplitude of the fundamental of the back-EMF of a trapezoidal
# waveform for one phase
phi = (2*sqrt(3)/pi)*K_p/p
# Mechanical speed at which the shaft is spinning
omega_mechanical_rpm = 5000
# Corresponding electrical frequency
omega_electrical_frequency_per_minute = p*omega_mechanical_rpm
# Fundamental of the phase current waveform
i_fundamental = (2*sqrt(3)/pi)*i_phase

# Can plug the above values into the circuit equation
# to get information about voltage, power, efficiency, etc.
V_phase = (j*omega_elecrical_frequency_per_minute*L_phase+R_phase_ohms)*i_fundamental+omega_electrical_frequency_per_minute*phi
#V_phase = 2.199552+0.741516i V
V_phase_for_required_torque = norm(V_phase)
#V_phase_for_required_torque = 2.321179V
# Line-to-line voltage if the machine is wye-configured.
# We could probably interpret this as the DC bus voltage
# for the two-phase-on system
V_delta = norm(sqrt(3)*V_phase)
#V_delta = 4.020401 V
if test == 1
	assert(V_delta,4.020401,0.0001)
end
# Real power = mechanical power + losses
P_real_W = 3/2*Re(V_phase*i_fund)
#P_real_W = 14.552117 W
if test == 1
	assert(P_real_W,14.552117,0.0001)
end
# Apparent power = volt*amp product that must
# actually be accommodated by the drive electronics
P_apparent = 3/2*norm(V_phase)*norm(i_fund)
tau_fundamental = 3/2*p*phi*i_fundamental
# Mechanical output power
P_mechanical_W = tau_fundamental*omega_mechanical_rpm
#P_mechanical_W = 13.113479 W
efficiency = P_mechanical_W/P_real_W
#efficiency = 0.901139

#-----------------
# Suggested Iron Cross-Section Sizing
#-----------------
# Air gap flux density
B_gap_Telsa = B_r_Telsa*(t_m_mm/(r_rotor_mm-r_stator_mm))
B_max_Telsa = 1.8
# Minimum "Leg" region iron thickness required to
# carry the flux from the permanent magnet.
w_leg_mm = w_m_mm * B_gap_Telsa / B_max_Telsa
#w_leg_mm = 3.359909
if test == 1
	assert(w_leg_mm,3.359909,0.0001)
end
# The back iron and rotor iron need about half the
# cross-section as the leg section to carry the flux
# from the magnets--in the case where all the flux
# goes down the "leg", half the flux goes to the left,
# and half the flux goes to the right...
w_backiron_mm = w_leg_mm/2
#w_backiron_mm = 1.179954

#-----------------
# Wire Sizing
#-----------------
i_rms = sqrt(2/3)*i_phase
# Current density in the wire. As a rule of thumb, it's
# good to stay below 10A/mm^2. However the maximum
# current density is really determined by how well
# the design can get the heat produced via resistive
# losses out
wire_current_density_A_per_mm2 = i_rms / a_wire
#wire_current_density_A_per_mm2 = 10.024119
if test == 1
	assert(wire_current_density_A_per_mm2,10.024119,0.0001)
end
if wire_current_density_A_per_mm2 > 10
	disp('wire_current_density_A_per_mm2 > 10, it is good to keep it below 10 as a rule of thumb to prevent heat buildup')
end
# Cross-section area of copper in each slot. As another
# rule of thumb, it's good to make sure that the copper
# area is less than half the total slot area so that
# the winding can actually be constructed.
copper_cross_section_area_mm2 = n*a_wire
#copper_cross_section_area_mm2 = 4.887192
if test == 1
	assert(copper_cross_section_area_mm2,4.887192,0.0001)
end