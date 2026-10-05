import numpy as np

def calculate_prop_coefficients():
# =========================================================================
    # 1. PARAMETRIC INPUTS (From your CSV and Prop Data)
    # =========================================================================
    # Wing Geometry
    b = 0.530                        # Wingspan (meters)
    sweep_deg = 15.0                 # Leading edge sweep angle (degrees)
    taper_ratio = 0.7                # Taper ratio (tip_chord / root_chord)
    c_root = 0.2049                  # Root chord (meters)
    
    # Elevon Geometry
    cf_c = 0.40                      # Elevon chord fraction (40% of local chord)
    # Assuming elevon (and motor) is centered on the semi-span:
    y_motor = (b / 2.0) / 2.0        # Spanwise location of motor/elevon center (b/4)
    
    # Propeller
    D_prop_in = 6.0                  # Propeller diameter in inches
    
    # Aerodynamic assumptions
    a_lift = 2.5                     # 3D lift slope of the immersed elevon
    k_emp = 0.60                     # Empirical efficiency loss (using 60% for 6x4.5 prop)

    # =========================================================================
    # 2. GEOMETRIC CALCULATIONS (Deriving d_roll and d_pitch)
    # =========================================================================
    sweep_rad = np.radians(sweep_deg)
    
    # --- Roll Arm ---
    d_roll = y_motor  # The roll lever arm is simply the spanwise distance to the motor
    
    # --- Local Wing Properties at Motor Location ---
    # Chord tapers linearly from root to tip
    c_tip = c_root * taper_ratio
    c_local = c_root - (c_root - c_tip) * (y_motor / (b / 2.0))
    
    # Local leading edge setback due to sweep
    x_le_local = y_motor * np.tan(sweep_rad)
    
    # Center of Pressure (X_CP) of the elevon at this local station
    # Hinge is at (1 - cf_c), and flap lift acts 25% behind the hinge
    x_cp_local = x_le_local + c_local * (1.0 - cf_c) + 0.25 * (c_local * cf_c)

    # --- Global Mean Aerodynamic Chord (MAC) and CG ---
    # Spanwise location of MAC
    y_mac = (b / 6.0) * ((1.0 + 2.0 * taper_ratio) / (1.0 + taper_ratio))
    
    # MAC length
    mac = (2.0 / 3.0) * c_root * ((1.0 + taper_ratio + taper_ratio**2) / (1.0 + taper_ratio))
    
    # X location of MAC leading edge
    x_le_mac = y_mac * np.tan(sweep_rad)
    
    # Design CG is located at 25% of the MAC
    x_cg = x_le_mac + 0.25 * mac
    
    # --- Pitch Arm ---
    d_pitch = x_cp_local - x_cg

    D_prop = D_prop_in * 0.0254
  
    
    # =========================================================================
    # 3. THIN AIRFOIL THEORY: FLAP EFFECTIVENESS (tau)
    # =========================================================================
    # Hinge angle theta_h based on flap chord fraction
    cos_theta_h = 1 - 2 * (1 - cf_c)
    theta_h = np.arccos(cos_theta_h)
    
    # Tau calculates how effective the elevon is compared to rotating the whole wing
    tau = 1 - (theta_h - np.sin(theta_h)) / np.pi

    # =========================================================================
    # 4. ACTUATOR DISK THEORY
    # =========================================================================
    # Propeller disk area
    A_p = np.pi * (D_prop / 2.0)**2
    
    # Slipstream contraction (the theoretical limit is 1/sqrt(2) of the prop diameter)
    D_slipstream = D_prop * (1.0 / np.sqrt(2.0))
    
    # Area of the elevon immersed in the slipstream
    c_elevon = c_local * cf_c
    S_imm = D_slipstream * c_elevon

    # =========================================================================
    # 5. Z-FORCE MULTIPLIER DERIVATION
    # =========================================================================
    # Dynamic pressure q_s = T / A_p
    # Force F_z = q_s * S_imm * (a_lift * tau * delta)
    # Substitute q_s: F_z = T * (S_imm / A_p) * a_lift * tau * delta
    # F_z = k_z_theoretical * T * delta
    k_z_theoretical = (S_imm / A_p) * a_lift * tau
    
    # Apply empirical efficiency factor
    k_z_real = k_z_theoretical * k_emp

    # =========================================================================
    # 6. MOMENT CONSTANTS (c_roll and c_pitch)
    # =========================================================================
    # Moment = Force * Lever Arm
    c_roll = k_z_real * d_roll
    c_pitch = k_z_real * d_pitch

    # =========================================================================
    # 7. OUTPUTS
    # =========================================================================
    print("--- ACTUATOR DISK PARAMETERS ---")
    print(f"Prop Area (A_p):          {A_p:.5f} m^2")
    print(f"Slipstream Dia:           {D_slipstream*1000:.1f} mm")
    print(f"Immersed Area (S_imm):    {S_imm:.5f} m^2")
    print(f"Flap Effectiveness (tau): {tau:.4f}")
    
    print("\n--- FORCE MULTIPLIER ---")
    print(f"Theoretical F_z / (T*d):  {k_z_theoretical:.4f}")
    print(f"Real-World F_z / (T*d):   {k_z_real:.4f}  (assuming {k_emp*100:.0f}% efficiency)")
    
    print("\n--- FINAL CONTROL CONSTANTS ---")
    print(f"c_roll  =  {c_roll:.4f}")
    print(f"c_pitch =  {c_pitch:.4f}")
    
    return c_roll, c_pitch

if __name__ == "__main__":
    calculate_prop_coefficients()