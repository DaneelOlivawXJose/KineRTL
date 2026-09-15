library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.robot_config_pkg.all;
use work.htm_pkg.all;

entity direct_kinematics_tb is
end entity direct_kinematics_tb;

architecture sim of direct_kinematics_tb is

    signal clk      : std_logic := '0';
    signal reset    : std_logic := '0';
    signal start    : std_logic := '0';
    signal done     : std_logic;
    signal sim_done : boolean := false;
    
    signal theta_in : array_of_fp(0 to DOF-1) := (others => (others => '0'));
    signal t_matrix : matrix_4x4_out;

    signal final_pos_x : fp_mult_type;
    signal final_pos_y : fp_mult_type;
    signal final_pos_z : fp_mult_type;

begin

    UUT: entity work.direct_kinematics
        generic map (
            ITERATIONS_COORDIC => ITERATIONS_CORDIC,
            DATA_WIDTH         => TOTAL_WIDTH
        )
        port map (
            clk      => clk,
            reset    => reset,
            start    => start,
            theta_in => theta_in,
            t_matrix => t_matrix,
            done     => done
        );

    final_pos_x <= t_matrix(0, 3);
    final_pos_y <= t_matrix(1, 3);
    final_pos_z <= t_matrix(2, 3);

    clk_process: process
    begin
        while not sim_done loop
            clk <= '0'; wait for 5 ns;
            clk <= '1'; wait for 5 ns;
        end loop;
        wait;
    end process;

    stim_process: process
    begin
        -- 1. Reset inicial
        reset <= '1';
        wait for 20 ns;
        reset <= '0';
        wait for 20 ns;

        -- =========================================================
        -- (Home Position - 0 rads)
        -- =========================================================
        theta_in(0) <= to_signed(0, TOTAL_WIDTH); 
        theta_in(1) <= to_signed(0, TOTAL_WIDTH); 
        theta_in(2) <= to_signed(0, TOTAL_WIDTH); 
        theta_in(3) <= to_signed(0, TOTAL_WIDTH); 
        theta_in(4) <= to_signed(0, TOTAL_WIDTH); 
        theta_in(5) <= to_signed(0, TOTAL_WIDTH); 
        
        start <= '1';
        wait for 10 ns;
        start <= '0';
        
        wait until done = '1';
        wait for 40 ns;

        -- =========================================================
        -- (90 degrees / 1.570796 rads)
        -- FRAC_WIDTH = 30, 1.570796 rad = 1.570796 * (2^30) ? 1686629713
        -- =========================================================
        theta_in(0) <= to_signed(1686629713, TOTAL_WIDTH); 
        
        start <= '1';
        wait for 10 ns;
        start <= '0';
        
        wait until done = '1';
        
        for i in 1 to 5 loop
            wait until rising_edge(clk);
        end loop;

        sim_done <= true;
        wait for 100 ns; 
        wait;
    end process;

end architecture sim;

-- ghdl -a robot_config_pkg.vhd
-- ghdl -a ../cordic/cordic.vhd
-- ghdl -a htm_pkg.vhd 
-- ghdl -a direct_kinematics.vhd               
-- ghdl -a direct_kinematics_tb.vhd            
-- ghdl -e direct_kinematics_tb                
-- ghdl -r direct_kinematics_tb --vcd=ondas.vcd --stop-time=50000ns