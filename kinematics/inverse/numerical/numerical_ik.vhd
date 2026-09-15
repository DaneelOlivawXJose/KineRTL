library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.robot_config_pkg.all;
use work.htm_pkg.all;

entity numerical_ik is
    generic(
        -- Tolerancia de error (ej: 1 mm escalado a Q8.24)
        MAX_ERR   : fp_type := to_signed(16777, TOTAL_WIDTH); 
        -- Delta para la derivada (~0.03 rad) -> shift de 5
        DELTA_VAL : fp_type := to_signed(524288, TOTAL_WIDTH);
        MAX_ITER  : integer := 30
    );
    port(
        clk       : in  std_logic;
        reset     : in  std_logic;
        start     : in  std_logic;
        theta_in  : in  array_of_fp(0 to DOF-1);
        pos_target: in  array_of_fp(0 to 2); -- [X, Y, Z]
        
        theta_out : out array_of_fp(0 to DOF-1);
        done      : out std_logic
    );
end entity numerical_ik;

architecture behavioral of numerical_ik is

    type FSM_state is (IDLE, START_DKS, WAIT_DKS, CALC_ERR_JAC, THETA_NEXT, FINISHED);
    signal state : FSM_state := IDLE;
    
    signal theta_curr : array_of_fp(0 to DOF-1);
    
    type array_of_theta_arrays is array (0 to DOF-1) of array_of_fp(0 to DOF-1);
    signal theta_perturbed : array_of_theta_arrays;
    
    signal t_mat_real : matrix_4x4_out;
    type array_of_matrices_out is array (0 to DOF-1) of matrix_4x4_out;
    signal t_mat_pert : array_of_matrices_out;
    
    signal err_x, err_y, err_z : fp_type;
    signal abs_err_total       : fp_type;
    
    type jacobian_type is array (0 to 2, 0 to DOF-1) of fp_type;
    signal jacobian : jacobian_type;

    signal start_all_dk  : std_logic := '0';
    signal dk_done_array : std_logic_vector(0 to DOF) := (others => '0');
    signal dk_done_latch : std_logic_vector(0 to DOF) := (others => '0');
    
    signal iter_count    : integer range 0 to 100 := 0;

begin

    -- Perturbaciones para el cálculo del Jacobiano
    process(theta_curr)
    begin
        for i in 0 to DOF-1 loop
            for j in 0 to DOF-1 loop
                if i = j then
                    theta_perturbed(i)(j) <= theta_curr(j) + DELTA_VAL;
                else
                    theta_perturbed(i)(j) <= theta_curr(j);
                end if;
            end loop;
        end loop;
    end process;

    -- Instanciación de módulos de Cinemática Directa (7 en total)
    GEN_DK_PERT: for i in 0 to DOF-1 generate
        DK_INST : entity work.direct_kinematics
            port map (
                clk       => clk,
                reset     => reset,
                start     => start_all_dk, 
                theta_in  => theta_perturbed(i),
                t_matrix  => t_mat_pert(i),
                done      => dk_done_array(i)
            );
    end generate GEN_DK_PERT;

    DK_REAL_INST : entity work.direct_kinematics
        port map (
            clk       => clk,
            reset     => reset,
            start     => start_all_dk, 
            theta_in  => theta_curr,
            t_matrix  => t_mat_real,
            done      => dk_done_array(DOF)
        );

    -- FSM de resolución numérica
    process(clk, reset)
        variable temp_err_x, temp_err_y, temp_err_z : fp_type;
        variable abs_x, abs_y, abs_z : fp_type;
        variable accum_theta : fp_mult_type;
    begin
        if reset = '1' then
            state         <= IDLE;
            start_all_dk  <= '0';
            done          <= '0';
            iter_count    <= 0;
            dk_done_latch <= (others => '0');
            for i in 0 to DOF-1 loop
                theta_curr(i) <= (others => '0');
                theta_out(i)  <= (others => '0');
            end loop;
            
        elsif rising_edge(clk) then
            start_all_dk <= '0'; 
            done         <= '0';
            
            case state is
                when IDLE =>
                    if start = '1' then
                        theta_curr    <= theta_in;
                        iter_count    <= 0;
                        dk_done_latch <= (others => '0');
                        state         <= START_DKS;
                    end if;

                when START_DKS =>
                    start_all_dk <= '1';
                    state        <= WAIT_DKS;

                when WAIT_DKS =>
                    -- Capturamos asincronías con un latch de seguridad
                    dk_done_latch <= dk_done_latch or dk_done_array;
                    
                    if unsigned(not (dk_done_latch or dk_done_array)) = 0 then
                        state <= CALC_ERR_JAC;
                    end if;

                when CALC_ERR_JAC =>
                    -- Lectura correcta de los 32 bits de datos útiles (TOTAL_WIDTH-1 downto 0)
                    temp_err_x := pos_target(0) - t_mat_real(0,3)(TOTAL_WIDTH-1 downto 0);
                    temp_err_y := pos_target(1) - t_mat_real(1,3)(TOTAL_WIDTH-1 downto 0);
                    temp_err_z := pos_target(2) - t_mat_real(2,3)(TOTAL_WIDTH-1 downto 0);
                    
                    err_x <= temp_err_x;
                    err_y <= temp_err_y;
                    err_z <= temp_err_z;
                    
                    abs_x := temp_err_x when temp_err_x >= 0 else -temp_err_x;
                    abs_y := temp_err_y when temp_err_y >= 0 else -temp_err_y;
                    abs_z := temp_err_z when temp_err_z >= 0 else -temp_err_z;
                    
                    abs_err_total <= abs_x + abs_y + abs_z;

                    for i in 0 to DOF-1 loop
                        jacobian(0, i) <= shift_right(t_mat_pert(i)(0,3)(TOTAL_WIDTH-1 downto 0) - t_mat_real(0,3)(TOTAL_WIDTH-1 downto 0), 5);
                        jacobian(1, i) <= shift_right(t_mat_pert(i)(1,3)(TOTAL_WIDTH-1 downto 0) - t_mat_real(1,3)(TOTAL_WIDTH-1 downto 0), 5);
                        jacobian(2, i) <= shift_right(t_mat_pert(i)(2,3)(TOTAL_WIDTH-1 downto 0) - t_mat_real(2,3)(TOTAL_WIDTH-1 downto 0), 5);
                    end loop;
                    
                    state <= THETA_NEXT;

                when THETA_NEXT =>
                    if abs_err_total < MAX_ERR or iter_count = MAX_ITER then
                        state <= FINISHED;
                    else
                        for i in 0 to DOF-1 loop
                            accum_theta := (jacobian(0, i) * err_x) + (jacobian(1, i) * err_y) + (jacobian(2, i) * err_z);
                            -- Factor de convergencia: Ajusta este 5 si diverge o es muy lento (4 a 7 es óptimo)
                            theta_curr(i) <= theta_curr(i) + resize(shift_right(accum_theta, FRAC_WIDTH + 5), TOTAL_WIDTH);
                        end loop;
                        
                        iter_count    <= iter_count + 1;
                        dk_done_latch <= (others => '0');
                        state         <= START_DKS;
                    end if;

                when FINISHED =>
                    theta_out <= theta_curr;
                    done      <= '1';
                    state     <= IDLE;
                    
            end case;
        end if;
    end process;
end architecture behavioral;