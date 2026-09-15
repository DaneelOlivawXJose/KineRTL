library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.robot_config_pkg.all;
use work.htm_pkg.all;

entity numerical_ik_tb is
end entity numerical_ik_tb;

architecture sim of numerical_ik_tb is

    -- Señales de reloj y control
    signal clk   : std_logic := '0';
    signal reset : std_logic := '1';
    signal start : std_logic := '0';
    
    -- Puertos del DUT (Device Under Test)
    signal theta_in   : array_of_fp(0 to DOF-1) := (others => (others => '0'));
    signal pos_target : array_of_fp(0 to 2)     := (others => (others => '0'));
    
    signal theta_out  : array_of_fp(0 to DOF-1);
    signal done       : std_logic;

    -- Periodo de reloj (ej. 27 MHz de la Tang Nano 9K -> ~37 ns)
    constant CLK_PERIOD : time := 37 ns;

begin

    -- Instanciación del módulo de Cinemática Inversa Numérica
    UUT : entity work.numerical_ik
        generic map (
            MAX_ERR   => to_signed(16777, TOTAL_WIDTH),  -- 1 mm de tolerancia
            DELTA_VAL => to_signed(524288, TOTAL_WIDTH), -- ~0.03 rad
            MAX_ITER  => 30
        )
        port map (
            clk        => clk,
            reset      => reset,
            start      => start,
            theta_in   => theta_in,
            pos_target => pos_target,
            theta_out  => theta_out,
            done       => done
        );

    -- Generador de reloj
    clk_process: process
    begin
        while true loop
            clk <= '0';
            wait for CLK_PERIOD / 2;
            clk <= '1';
            wait for CLK_PERIOD / 2;
        end loop;
    end process;

    -- Proceso de estímulos principal
    stimulus_process: process
    begin
        -- 1. Reset del sistema
        reset <= '1';
        start <= '0';
        wait for CLK_PERIOD * 5;
        reset <= '0';
        wait for CLK_PERIOD * 5;

        -- 2. Configurar Test 1: Mover desde origen (Home) a la postura de 90 grados
        -- Posición inicial (theta_in): Todo ceros.
        for i in 0 to DOF-1 loop
            theta_in(i) <= to_signed(0, TOTAL_WIDTH);
        end loop;

        -- Posición objetivo (X, Y, Z) en Q8.24 (Valores extraídos de tu Test 2)
        -- X = -1768938 (~0 m), Y = 10153412 (~0.6 m), Z = 25451994 (~1.517 m)
        pos_target(0) <= to_signed(-1768938, TOTAL_WIDTH); 
        pos_target(1) <= to_signed(10153412, TOTAL_WIDTH);
        pos_target(2) <= to_signed(25451994, TOTAL_WIDTH);

        -- Disparar el cálculo
        wait until rising_edge(clk);
        start <= '1';
        wait until rising_edge(clk);
        start <= '0';

        -- 3. Esperar a que converja el algoritmo
        wait until done = '1';
        
        -- Dejar un margen visual en la simulación
        wait for CLK_PERIOD * 20;

        -- 4. Fin de la simulación
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
-- ghdl -r --std=08 numerical_ik_tb --vcd=ondas.vcd --stop-time=50000ns