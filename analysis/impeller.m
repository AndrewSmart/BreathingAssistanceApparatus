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

function pressure_flow_curve = impeller(B1_degrees,B2_degrees)
	% Models characteristics of an impeller with B1_degrees and B2_degrees.
	format short g;
	pressure_flow_curve = []
	#for rpm=3000:400:5400
	rpm=2427
		pressure_flow_curve = [pressure_flow_curve; characterize_impeller(rpm,B1_degrees,B2_degrees)];
	#end
end

function pressure_flow = tune(B2_degrees)
	rpm = 2427
	B1_degrees=46
	pressure_target=16
	#Outer loop b2, inner loop solve for b1 for 4,8,12,16,20 setpoints with .0032 flow
	while(true)
		pressure_flow = characterize_impeller(rpm,B1_degrees,B2_degrees)
		if(pressure_flow(2) > 0.00321)
			rpm -= 2
		elseif(pressure_flow(2) < 0.00319)
			rpm += 2
		end
		if (pressure_flow(3)-pressure_target > 0.01)
			B1_degrees += 0.002
		elseif (pressure_flow(3)-pressure_target < 0.01)
			B1_degrees -= 0.002
		end
		if (abs(pressure_flow(3)-pressure_target) < 0.01 && abs(pressure_flow(2)-0.0032) < 0.00001)
			break
		end
	end
end
#b1,b2,rpm,flow,pressure,power_watts
#b1=64,b2=25 2427   0.0032008      4.0007    -0.13449
#b1=56,b2=25 3400   0.0032423       8.008    -0.27271
#b1=51,b2=25 4150   0.0032964      12.032    -0.41659
#b1=46.2,b2=25 4770   0.0031995          16    -0.53768
#b1=43,b2=25 5350    0.003209      20.203    -0.68094

#b1=46.39,b2=45 4726   0.0031911      16.001     -0.5363

#b1=43.13,b2=25, 5324   0.0032086      20.004    -0.67414
#b1=43.28,b2=35, 5295   0.0032084      19.994    -0.67376
#b1=43.365,b2=45, 5280   0.0032083      20.005    -0.67411
#No efficency change at pressure/flow points, because torque increases with decreasing rpm.
#increasing b2 has decreasing slope of flow/pressure. e.g. if b2=135 and impeller designed for
# pressure=12, then flow won't change as much when adjusting pressure.

function pressureflow = characterize_impeller(rpm,B1_degrees,B2_degrees)
	losses = 0; #Set to 0 if we want ideal design results. 1 to calculate what few losses we do calculate.
	deg_to_rad_coeff = (pi)/180;
	g_m_per_s2 = 9.80665;
	cmH2O_to_pascal_coeff = 98.0665;
	#--------------------
	# Pressure setting
	ventilator_pressure_setting_cmH2O = 20
	ventilator_pressure_setting_Pascals = ventilator_pressure_setting_cmH2O * cmH2O_to_pascal_coeff;

	#--------------------
	# Density of air calculation
	# https://en.wikipedia.org/wiki/Density_of_air
	# Skipped humidity for now
	p0_sea_level_air_pressure_Pascals = 101325;
	T0_sea_level_standard_temperature_K = 288.15;
	g_earth_surface_gravitational_acceleration_m_per_s2 = 9.80665;
	L_temperature_lapse_rate_K_per_m = 0.0065;
	R_ideal_universal_gas_constant_Joules_per_mol_K = 8.31447;
	M_molar_mass_dry_air_kg_per_mol = 0.0289644;
	#Altitude my town
	h_altitude_meters = 1500;
	p_pressure_at_altitude_Pascals = p0_sea_level_air_pressure_Pascals * (1-(L_temperature_lapse_rate_K_per_m*h_altitude_meters)/T0_sea_level_standard_temperature_K)^((g_earth_surface_gravitational_acceleration_m_per_s2*M_molar_mass_dry_air_kg_per_mol)/(R_ideal_universal_gas_constant_Joules_per_mol_K*L_temperature_lapse_rate_K_per_m));
	# Could either get this from sensor reading on startup or UI entry of altitude
	p_ambient_air_pressure_Pascals = p_pressure_at_altitude_Pascals
	temperature_K = 293;
	R_specific_constant_dry_air_Joules_per_kg_K = 287.058;
	outlet_pressure_Pascals = p_ambient_air_pressure_Pascals + ventilator_pressure_setting_Pascals
	rho_ambient_air_density_kg_per_m3 = p_ambient_air_pressure_Pascals / (R_specific_constant_dry_air_Joules_per_kg_K*temperature_K)
	rho_ventilator_air_density_kg_per_m3 = outlet_pressure_Pascals / (R_specific_constant_dry_air_Joules_per_kg_K*temperature_K)
	#rho_ambient_air_density_kg_per_cm3 = rho_ambient_air_density_kg_per_m3 * 1e-6;
	C_p_specific_heat_air_kJ_per_kg_K = 1.005; #https://www.ohio.edu/mechanical/thermo/property_tables/air/air_Cp_Cv.html

	# Now calculate pressurized air density
	rho_ventilator_pressure_air_density_kg_per_m3 = outlet_pressure_Pascals / (R_specific_constant_dry_air_Joules_per_kg_K*temperature_K);
	#rho_ventilator_pressure_air_density_kg_per_cm3 = rho_ventilator_pressure_air_density_kg_per_m3 * 1e-6;
	pressure_ratio = (p_ambient_air_pressure_Pascals+ventilator_pressure_setting_Pascals) / p_ambient_air_pressure_Pascals

	#---------------------
	# Design variables
	# angle of blades at inlet, careful about measurement! See figure 1.3 in Principles of Turbomachinery
	# B1 approaching 90 degrees greatly increases Q and torque
	#B1_degrees = 43.4;
	B1_rads = B1_degrees * deg_to_rad_coeff;
	# angle of blades at outlet, careful this is measured opposite u, not inside triangle, see diagram
	#  Should be at or near 90 degrees, as this way head (pressure) will not vary with flow.
	#  As we'd need more flow more larger lungs, without changing the pressure setpoint.
	#B2_degrees = 45;
	B2_ideal_rads = B2_degrees * deg_to_rad_coeff;
	# A1 is angle fluid enters the impeller, for best efficiency assumed 90 degrees
	A1_degrees = 90;
	A1_rads = A1_degrees * deg_to_rad_coeff;
	C1_rads = pi - A1_rads - B1_rads;
	#rpm=5400
	# b1 is the width of the flowpassage at inlet,perpendicular to plane
	b1_m = 0.010;
	# b2 is the width of the flowpassage at outlet, perpendicular to plane
	# Decreasing this is the principal way of increasing A2 and W2
	b2_m = 0.010;

	w_rad_per_min=rpm*2*pi;
	w_rad_per_sec=w_rad_per_min/60;
	r1_m = 0.01; #radius of hub.
	r2_m = 0.08;
	dn_m = 2 * r1_m; #diameter of hub.
	d1a_m = 2 * (r1_m + b1_m); #diameter of inlet.
	# Turton - "Principles of Turbomachinery" §6.3.2 eqn 6.3-6.4
	# Z = B2/3 B2 in degrees
	# Z = 2*pi*sin(B_m)/(.35 to .45)log_e(D2/D1)
	Z1 = B2_degrees / 3
	Z2 = 2 * pi * sin((B2_ideal_rads+B1_rads)/2)/(.35*log(r2_m/r1_m))
	N_Blades = 9;

	#--------------------
	# Inlet Triangle
	# inlet impeller tip speed
	# U is perepheral velocity vector
	U1_m_per_sec = w_rad_per_sec * r1_m
	# Impeller tip speed
	U2_m_per_sec = w_rad_per_sec * r2_m
	# U2 = pi*D[m]*N[rpm]/60[s]

	# See Principles of Turbomechanics Exercise 1.1
	# Urgh in Fluid Mechanics and Turbo Design ch2 replace V with W, and C with V. Cw with Vu, and Cr with Vr

	# W is relative velocity vector (of the fluid passing through the impeller passages)
	# b is angle between W and V's origin (blade angle)
	W1_m_per_sec = U1_m_per_sec*sin(A1_rads)/sin(C1_rads)
	# V is absolute velocity vector, sometimes labeled as C in textbooks
	# a is angle between U and V's origin
	# From Principles of Turbomachinery Figure 1.4 and Law of Cosines (https://academic.evergreen.edu/projects/biophysics/technotes/misc/trig.htm)
	#V1_cm_per_sec = sqrt(W1_cm_per_sec^2+U1_cm_per_sec^2-2*W1_cm_per_sec*U1_cm_per_sec*cos(B1_rads))
	# But I think sin/cos forumla is more efficient/simpler
	V1_m_per_sec = U1_m_per_sec*sin(B1_rads)/sin(C1_rads)
	#Vr is notated as Vm in Centrifugal and Rotary Pumps Fundamentals and Applications, Vm is called a "meriodional" velocity (a component of the absolute velocity into the meriodional direction)
	Vr1_m_per_sec = sin(A1_rads) * V1_m_per_sec
	# As A1 is 90 degrees, at inlet Vr1=V1, and Vu1=0
	# Vu is the component of absolute velocity in the tangental direction
	# As A1=90 then Vu1=0, but for completion lets compute Vu1 in the event a1 isn't 90 degrees
	Vu1_m_per_sec= cos(A1_rads) * V1_m_per_sec
	if A1_degrees == 90
		assert(Vu1_m_per_sec,0,0.0001)
	end

	# Flude Mechanics and Turbo Design section 2.2
	# (V2^2-V1^2)/2 represents the change in kinetic energy of the liquid
	# (U2^2-U1^2)/2 represents the effect of the centrifugal head or energy produced by the impeller
	# (W2^2-W1^2)/2 represents the change in static pressure of the liquid, if the losses in the impeller are neglected
	# But Principles of TurboMachinery eqn 1.10 states somewhat different formulas somewhat differently, esp W.
	# From Centrifugal and Rotary Pumps Fundamentals and Applications eqn 26,27
	# Vu = U - W * cos(b)
	# Wu = W * cos(b) = U - Vu 

	#Lung tidal + reserve volume is 4.8L for males, adult breaths per minute is 12-20.
	lung_volume_male_liters = 4.8;
	breaths_max_per_minute = 20;
	breath_time_seconds = 60 / breaths_max_per_minute;
	inhalation_time_seconds = breath_time_seconds / 2;
	inhalation_liters_per_second = lung_volume_male_liters / inhalation_time_seconds;
	Qtarget_m3_per_sec = inhalation_liters_per_second / 1000

	#---------------------
	# Flow Coefficient
	# Swain - "IMPACT OF IMPELLER BLADE TRIMMING ON THE PERFORMANCE OF CENTRIFUGAL COMPRESSORS" §1.2.2,§2.1
	# Wang et al. - "Design and Performance Evaluation of a Very Low Flow Coefficient Centrifugal Compressor"
	# TODO: Swain clams it is D2, but Wang et al claims it is D2^2
	Flow_Coefficient_Swain = 4 * Qtarget_m3_per_sec/(pi*(r2_m*2)*U2_m_per_sec);
	f_q = 1; #Impeller inlet eyes
	flow_coefficient_inlet = 4 * Qtarget_m3_per_sec / (f_q * pi * (d1a_m^2-dn_m^2)*U1_m_per_sec)

	#---------------------
	# Flow calculation
	#  One book took off the blade area, other two did not. The book also covered many efficiency loss formulas while the other two did not.
	# Am1 is the area of the impeller at inlet in the [m]eriodional direction
	Am1_m2 = 2 * pi * r1_m * b1_m;
	# Am2 is the area of the impeller at outlet
	Am2_m2 = 2 * pi * r2_m * b2_m;

	# Q_in is the volume flow rate entering the impeller
	Q_in_m3_per_sec = Am1_m2 * Vr1_m_per_sec
	# m_dot is the mass flow rate
	m_dot_mass_flow_rate_kg_per_sec = rho_ambient_air_density_kg_per_m3 * Q_in_m3_per_sec
	# Q_out is the volume flow rate leaving the impeller, must take into account different pressure
	# TODO: I may have made an error here by "setting" outlet pressure and not determinining it by impeller chararcteristics
	#Q_out_cm3_per_sec = A2_cm2 * Vr2_cm_per_sec
	Q_out_m3_per_sec = m_dot_mass_flow_rate_kg_per_sec / rho_ventilator_pressure_air_density_kg_per_m3

	#--------------------
	# Outlet Triangle
	# Swain - "IMPACT OF IMPELLER BLADE TRIMMING ON THE PERFORMANCE OF CENTRIFUGAL COMPRESSORS" §1.2.4
	# Swain shows a popular slip model proposed by Stodola (and alludes to a better jet/wake model).
	# Turton - "Principles of Turbomachinery" §5.3.2 also cites Stodola but the eqn is different...:
	Vus_m_per_sec = (pi * U2_m_per_sec * cos(B2_ideal_rads)) / N_Blades
	Vr2_m_per_sec = Q_out_m3_per_sec / Am2_m2
	Wu2_ideal_m_per_sec = Vr2_m_per_sec * cot(B2_ideal_rads)
	Vu2_ideal_m_per_sec = U2_m_per_sec - Wu2_ideal_m_per_sec
	if (losses==1)
		Vu2_m_per_sec = Vu2_ideal_m_per_sec - Vus_m_per_sec
		Wu2_m_per_sec = U2_m_per_sec - Vu2_m_per_sec
		# TODO: Derive ideal/actual B2_rads and A2_rads
		B2_slip_rads = acot(Wu2_m_per_sec/Vr2_m_per_sec);
		#W2_cm_per_sec = sqrt(U2_cm_per_sec^2+V2_cm_per_sec^2 - 2*U2_cm_per_sec*V2_cm_per_sec * cos(A2_rads))
		# Simpler W2 eqn 26 from Centrifugal and Rotary Pumps Fundamentals
		W2_m_per_sec = Vr2_m_per_sec / sin(B2_slip_rads)
	else
		Vu2_m_per_sec = Vu2_ideal_m_per_sec
		Wu2_m_per_sec = Wu2_ideal_m_per_sec
		W2_m_per_sec = Vr2_m_per_sec / sin(B2_ideal_rads)
	endif
	A2_rads = atan(Vr2_m_per_sec / Vu2_m_per_sec);
	A2_degrees = A2_rads / deg_to_rad_coeff
	#V2_cm_per_sec = sqrt(Vr2_cm_per_sec^2+Vu2_cm_per_sec^2)
	V2_m_per_sec = Vr2_m_per_sec / sin(A2_rads)

	#--------------------
	# Theoretical Head Calculation
	# Principles of Turbomachinery gH_E is the work done per unit mass, eqn 1.3
	# Principles of Turbomachinery eqn 1.9, 1.10, and 1.11
	#  gH_E = u2*Vu2 - u1*Vu1
	# Or alternatively
	#  gH_E = sqrt(V2^2-V1^2)+(u2^2-u1^2)+(W1^2-W2^2))
	#gH_E = U2_cm_per_sec *Vu2_cm_per_sec-U1_cm_per_sec*Vu1_cm_per_sec
	gH_E_m2_per_s2 = U2_m_per_sec * Vu2_m_per_sec - U1_m_per_sec * Vu1_m_per_sec
	# m^2/s^2 is the same units as J/kg
	gH_E_J_per_kg = gH_E_m2_per_s2
	H_m = U2_m_per_sec * Vu2_m_per_sec / g_m_per_s2

	#--------------------
	# Delta Pressure
	delta_pressure_J_per_m3 = gH_E_J_per_kg * rho_ambient_air_density_kg_per_m3;
	delta_pressure_N_per_m2 = delta_pressure_J_per_m3;
	delta_pressure_Pascals = delta_pressure_J_per_m3;
	delta_pressure_Psi = delta_pressure_Pascals * 0.000145038;
	delta_pressure_cmH2O = delta_pressure_Pascals / cmH2O_to_pascal_coeff

	#--------------------
	# Torque/Power Needed
	# Fluid Mechanics and Turbo Design
	# Power_Delivered_J_per_s = density*g*Q*H
	# Principles of Turbomachinery Exercise 1.1
	# Hydrolic Power = rho*Q_volume_flow*gH_E
	# But for any fluid formula will be
	# Hydrolic Power = m_dot*gH_E
	hydrolic_power_J_per_sec = m_dot_mass_flow_rate_kg_per_sec * gH_E_J_per_kg;
	hydrolic_power_W = hydrolic_power_J_per_sec;

	# T_Nm = Power/angular velocity (Fluid Mechanics and Turbo Design)
	# m_dot is the mass flow rate
	# T is torque in kg*m^2/s^2 (or Nm as N=kgm/s^2)
	# T = m_dot * (Vu1*r1-Vu2*r2)
	# Swain - "IMPACT OF IMPELLER BLADE TRIMMING ON THE PERFORMANCE OF CENTRIFUGAL COMPRESSORS" §1.2.4
	# T = (m_dot + m_dot_fluid_leakage) * (cu2*r2 - cu1*r1) + T_F_torque_input_adsorbed_by_disk_friction
	Torque_Nm = m_dot_mass_flow_rate_kg_per_sec * (Vu1_m_per_sec*r1_m - Vu2_m_per_sec*r2_m);
	P_rot = Torque_Nm * w_rad_per_sec;
	P_rot_To_Watts_coeff = 0.1047;
	P_rot_watts = P_rot * P_rot_To_Watts_coeff;
	#P_loss_J_per_s = .2 * P_rot_watts; # assume 80% we're at efficiency on motor load curve
	#P_electrical_watts = P_rot_watts + P_loss_J_per_s;
	#P_electrical_amps = P_electrical_watts / 12;

	#--------------------
	# Impeller Height Calculation
	# Get z/D2 ratio, solve for z (optimal axial length)
	# Online calculator lookup is 1.9500cm
	z_d2 = (0.084375 + 1.5625 * flow_coefficient_inlet)
	z_m = z_d2 * (2 * r2_m)
	# TODO! Should Blade length be deduced from B1,B2,r1,r2,and a_m? Or pulled out of CAD model?
	# For now approximate as triangle at base of blade... seems to be a good approximation but doesn't
	# consider B1/B2 skew.
	Blade_length_m = sqrt(z_m^2+(r2_m-r1_m)^2)

	#--------------------
	# 1D Loss Calculation
	# Patel, Joshi - "Effect of Impeller Blade Exit Angle on the Performance of Centrifugal Pump"
	# Won't be as accurate as 2D, pseudo 3D, or 3D modeling, but appears reasonably close.
	# Hydraulic Losses in Impeller
	# loss_impeller = loss_impller_friction_mixing + loss_impeller_shock
	W_average_m_per_sec = (W1_m_per_sec + W2_m_per_sec)/2;
	# TODO: Coefficient of dissipation? Is this the diffusion ratio?
	C_d = 1;
	# TODO: This was a guess, need example
	Diameter_hydralic = (r1_m+b1_m)*2;
	loss_impeller_friction_mixing = 4*C_d*(Blade_length_m/Diameter_hydralic)*(W_average_m_per_sec/U2_m_per_sec)^2
	# TODO: I don't know what inlet throat means..., or inlet mean...
	loss_impeller_shock = 0.3*(W1_m_per_sec/U2_m_per_sec)^2
	loss_impeller = loss_impeller_friction_mixing + loss_impeller_shock

	pressureflow = [rpm,Q_out_m3_per_sec,delta_pressure_cmH2O,P_rot_watts];
end
