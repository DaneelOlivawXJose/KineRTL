library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.robot_config_pkg.all;

package htm_pkg is
    type matrix_4x4_in  is array (0 to 3, 0 to 3) of fp_type;
    type matrix_4x4_out is array (0 to 3, 0 to 3) of fp_mult_type;

    type array_of_matrices is array (natural range <>) of matrix_4x4_in;
end package htm_pkg;