library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.robot_config_pkg.all;
use work.htm_pkg.all;

entity numerical_ik_tb is
end entity numerical_ik_tb;

architecture sim of numerical_ik_tb is

    -- Señales de reloj y control globales
    signal clk   : std_logic := '0';
    signal reset : std_logic := '1';
    
    -- Señales para el Generador de Objetivo (DK)
    signal start_dk  : std_logic := '0';
    signal done_dk   : std_logic;
    signal theta_tgt : array_of_fp(0 to DOF-1) := (others => (others => '0'));
    signal t_mat_out : matrix_4x4_out;
    
    -- Puertos del DUT (IK)
    signal start_ik   : std_logic := '0';
    signal theta_in   : array_of_fp(0 to DOF-1) := (others => (others => '0'));
    signal pos_target : matrix_4x4_in;
    
    signal theta_out  : array_of_fp(0 to DOF-1);
    signal done_ik    : std_logic;

    constant CLK_PERIOD : time := 37 ns;

begin

    -- =========================================================================
    -- 1. GENERADOR DE OBJETIVO (Cinemática Directa)
    -- =========================================================================
    TARGET_GENERATOR : entity work.direct_kinematics
        port map (
            clk       => clk,
            reset     => reset,
            start     => start_dk,
            theta_in  => theta_tgt,
            t_matrix  => t_mat_out,
            done      => done_dk
        );

    -- Conversión combinacional: De matriz 64-bit (salida DK) a 32-bit (entrada IK)
    process(t_mat_out)
    begin
        for i in 0 to 3 loop
            for j in 0 to 3 loop
                pos_target(i,j) <= t_mat_out(i,j)(TOTAL_WIDTH-1 downto 0);
            end loop;
        end loop;
    end process;

    -- =========================================================================
    -- 2. DISPOSITIVO BAJO PRUEBA (IK 6-DOF)
    -- =========================================================================
    UUT : entity work.numerical_ik
        generic map (
            MAX_ERR   => to_signed(16777, TOTAL_WIDTH),  -- 1 mm / rad escalado
            DELTA_VAL => to_signed(524288, TOTAL_WIDTH), -- ~0.03 rad
            MAX_ITER  => 300 -- Le damos más margen al 6-DOF
        )
        port map (
            clk        => clk,
            reset      => reset,
            start      => start_ik,
            theta_in   => theta_in,
            pos_target => pos_target,
            theta_out  => theta_out,
            done       => done_ik
        );

    clk_process: process
    begin
        while true loop
            clk <= '0';
            wait for CLK_PERIOD / 2;
            clk <= '1';
            wait for CLK_PERIOD / 2;
        end loop;
    end process;

    stimulus_process: process
    begin
        reset <= '1';
        start_dk <= '0';
        start_ik <= '0';
        wait for CLK_PERIOD * 5;
        reset <= '0';
        wait for CLK_PERIOD * 5;

        -- =====================================================================
        -- CONFIGURACIÓN DEL TEST
        -- =====================================================================
        -- 1. Ángulos objetivo (Lo que tiene que adivinar el IK)
        theta_tgt(0) <= to_signed(26354460, TOTAL_WIDTH);  -- J1 = 90 deg
        theta_tgt(1) <= to_signed(4392410, TOTAL_WIDTH);   -- J2 = 15 deg
        theta_tgt(2) <= to_signed(-4392410, TOTAL_WIDTH);  -- J3 = -15 deg
        theta_tgt(3) <= to_signed(13177230, TOTAL_WIDTH);  -- J4 = 45 deg
        theta_tgt(4) <= to_signed(8784820, TOTAL_WIDTH);   -- J5 = 30 deg
        theta_tgt(5) <= to_signed(26354460, TOTAL_WIDTH);  -- J6 = 90 deg

        -- 2. Ángulos de inicio (Punto de partida del algoritmo)
        theta_in(0) <= to_signed(13177230, TOTAL_WIDTH); -- Empieza a 45 grados
        theta_in(1) <= to_signed(2000000, TOTAL_WIDTH);
        theta_in(2) <= to_signed(-2000000, TOTAL_WIDTH);
        theta_in(3) <= to_signed(6000000, TOTAL_WIDTH);
        theta_in(4) <= to_signed(4000000, TOTAL_WIDTH);
        theta_in(5) <= to_signed(13177230, TOTAL_WIDTH);

        -- =====================================================================
        -- EJECUCIÓN
        -- =====================================================================
        -- Generar la Matriz Objetivo ejecutando el DK
        wait until rising_edge(clk);
        start_dk <= '1';
        wait until rising_edge(clk);
        start_dk <= '0';
        
        wait until done_dk = '1';
        wait for CLK_PERIOD * 2;

        -- Disparar el solucionador IK
        wait until rising_edge(clk);
        start_ik <= '1';
        wait until rising_edge(clk);
        start_ik <= '0';

        -- Esperar a que converja el IK
        wait until done_ik = '1';
        
        wait for CLK_PERIOD * 20;

        assert false report "SIMULATION FINISHED CORRECTLY" severity failure;
        
    end process;

end architecture sim;

-- ghdl -a --std=08 ../../../robot_config_pkg.vhd
-- ghdl -a --std=08 ../../../common/cordic/cordic.vhd
-- ghdl -a --std=08 ../../forward/htm_pkg.vhd
-- ghdl -a --std=08 ../../../common/matrix_ops/mult_m.vhd 
-- ghdl -a --std=08 ../../forward/direct_kinematics.vhd
-- ghdl -a --std=08 numerical_ik.vhd           
-- ghdl -a --std=08 numerical_ik_tb.vhd            
-- ghdl -e --std=08 numerical_ik_tb                
-- ghdl -r --std=08 numerical_ik_tb --fst=ondas_fst.fst --stop-time=5000000ns