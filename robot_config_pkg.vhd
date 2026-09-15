library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package robot_config_pkg is

    -- Constant parameters for fixed-point representation
    constant TOTAL_WIDTH : natural := 32;
    constant FRAC_WIDTH  : natural := 24;
    constant ITERATIONS_CORDIC : natural := 26;

    subtype fp_type      is signed(TOTAL_WIDTH-1 downto 0);
    subtype fp_mult_type is signed((TOTAL_WIDTH*2)-1 downto 0);
    constant FP_ONE  : fp_type := to_signed(2**FRAC_WIDTH, TOTAL_WIDTH);
    constant FP_ZERO : fp_type := to_signed(0, TOTAL_WIDTH);
    constant FP_HALF_MULT : fp_mult_type := to_signed(2**(FRAC_WIDTH-1), TOTAL_WIDTH*2);
    
    -- Hardcoded alpha values for Denavit-Hartenberg parameters
    type alpha_enum is (A_0, A_90, A_180, A_MINUS_90);
    
    -- DH parameters for the robot (scaled to fixed-point representation)
    type dh_param_type is record
        a     : fp_type; 
        d     : fp_type; 
        alpha : alpha_enum;          
    end record;

    type dh_rom_type is array (natural range <>) of dh_param_type;
    
    type array_of_fp        is array (natural range <>) of fp_type;
    type array_of_std_logic is array (natural range <>) of std_logic;

    -- Hardcoded DH parameters for the robot (scaled to fixed-point representation)
    -- MUST CHANGE FOR DIFFERENT ROBOT CONFIGURATIONS
    constant ROBOT_ROM : dh_rom_type := (
        0 => (a => to_signed(687735, TOTAL_WIDTH),   d => to_signed(10994384, TOTAL_WIDTH), alpha => A_90),
        1 => (a => to_signed(8656762, TOTAL_WIDTH),  d => to_signed(0, TOTAL_WIDTH),          alpha => A_0),
        2 => (a => to_signed(961862, TOTAL_WIDTH),   d => to_signed(0, TOTAL_WIDTH),          alpha => A_MINUS_90),
        3 => (a => to_signed(0, TOTAL_WIDTH),        d => to_signed(10030832, TOTAL_WIDTH), alpha => A_90),
        4 => (a => to_signed(0, TOTAL_WIDTH),        d => to_signed(0, TOTAL_WIDTH),          alpha => A_MINUS_90),
        5 => (a => to_signed(0, TOTAL_WIDTH),        d => to_signed(4426770, TOTAL_WIDTH),  alpha => A_0)
    );

    constant DOF : integer := ROBOT_ROM'length;

end package robot_config_pkg;