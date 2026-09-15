-- ==============================================================================
-- @file    numerical_ik.vhd
-- @brief   N-DOF Numerical Inverse Kinematics Co-processor
-- @author  Jose Segura Montes
-- @date    15/09/2026
--
-- @note    ALGORITHM DESCRIPTION:
--          Uses an iterative Jacobian Transpose method via Finite Differences.
--          Orientation error is calculated using vector cross-products of the 
--          rotation matrix columns (Normal, Orientation, Approach) to avoid 
--          heavy trigonometric functions (CORDIC) and Gimbal Lock.
--
-- @note    HARDWARE ARCHITECTURE:
--          Calculates the 6xN Jacobian matrix on-the-fly by instantiating 
--          parallel Direct Kinematics (DK) pipelines.
--          Data format: Q8.24 Signed Fixed-Point Arithmetic (Can be modified in robot_config_pkg.vhd).
-- ==============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.robot_config_pkg.all;
use work.htm_pkg.all;

entity numerical_ik is
    generic(
        -- Maximum allowed error (Spatial & Angular combined)
        -- Scaled to Q8.24 (e.g., 16777 represents ~1 mm or ~0.001 rad)
        MAX_ERR   : fp_type := to_signed(16777, TOTAL_WIDTH); 
        
        -- Delta perturbation for Finite Difference derivative (~0.03 rad)
        DELTA_VAL : fp_type := to_signed(524288, TOTAL_WIDTH);
        
        -- Failsafe limit to prevent infinite loops in local minima
        MAX_ITER  : integer := 150
    );
    port(
        clk       : in  std_logic;
        reset     : in  std_logic;
        
        -- Control & Input Data
        start     : in  std_logic;
        theta_in  : in  array_of_fp(0 to DOF-1);  -- Initial joint angles (seed)
        pos_target: in  matrix_4x4_in;            -- 6-DOF Target Homogeneous Matrix
        
        -- Output Data
        theta_out : out array_of_fp(0 to DOF-1);  -- Solved joint angles
        done      : out std_logic
    );
end entity numerical_ik;

architecture behavioral of numerical_ik is

    -- ==========================================================================
    -- @brief Q8.24 Fixed-Point Multiplier Helper
    -- Performs multiplication and resizes back to standard bus width.
    -- ==========================================================================
    function fp_mult(a : fp_type; b : fp_type) return fp_type is
        variable prod : fp_mult_type;
    begin
        prod := a * b;
        return resize(shift_right(prod, FRAC_WIDTH), TOTAL_WIDTH);
    end function;

    -- Finite State Machine definition
    type FSM_state is (IDLE, START_DKS, WAIT_DKS, CALC_ERR_JAC, THETA_NEXT, FINISHED);
    signal state : FSM_state := IDLE;
    
    -- Joint angles registers
    signal theta_curr : array_of_fp(0 to DOF-1);
    
    -- Perturbed angles for Jacobian calculation
    type array_of_theta_arrays is array (0 to DOF-1) of array_of_fp(0 to DOF-1);
    signal theta_perturbed : array_of_theta_arrays;
    
    -- Matrices from DK outputs
    signal t_mat_real : matrix_4x4_out;
    type array_of_matrices_out is array (0 to DOF-1) of matrix_4x4_out;
    signal t_mat_pert : array_of_matrices_out;
    
    -- 6D Error Vectors
    signal err_x,  err_y,  err_z  : fp_type;
    signal err_rx, err_ry, err_rz : fp_type;
    signal abs_err_total          : fp_type;
    
    -- Generalized 6xN Jacobian Matrix
    type jacobian_type is array (0 to 5, 0 to DOF-1) of fp_type;
    signal jacobian : jacobian_type;

    -- Synchronization signals for parallel DK instances
    signal start_all_dk  : std_logic := '0';
    signal dk_done_array : std_logic_vector(0 to DOF) := (others => '0');
    signal dk_done_latch : std_logic_vector(0 to DOF) := (others => '0');
    
    signal iter_count    : integer range 0 to 500 := 0;

begin

    -- ==========================================================================
    -- 1. PERTURBATION GENERATOR
    -- Creates N arrays of joint angles, where the i-th array has the i-th 
    -- joint perturbed by DELTA_VAL. Used for numerical derivation.
    -- ==========================================================================
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

    -- ==========================================================================
    -- 2. DIRECT KINEMATICS (DK) PIPELINES
    -- Parallel instantiation to compute actual and perturbed matrices in hardware
    -- ==========================================================================
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

    -- ==========================================================================
    -- 3. NUMERICAL SOLVER FSM (Jacobian Transpose)
    -- ==========================================================================
    process(clk, reset)
        variable temp_err_x, temp_err_y, temp_err_z : fp_type;
        variable temp_err_rx, temp_err_ry, temp_err_rz : fp_type;
        variable accum_theta : fp_mult_type;
        
        -- Temporary variables for Normal (n), Orientation (o), and Approach (a) vectors
        variable n_cx, n_cy, n_cz, o_cx, o_cy, o_cz, a_cx, a_cy, a_cz : fp_type;
        variable n_tx, n_ty, n_tz, o_tx, o_ty, o_tz, a_tx, a_ty, a_tz : fp_type;
        variable n_px, n_py, n_pz, o_px, o_py, o_pz, a_px, a_py, a_pz : fp_type;
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
                    -- Asynchronous capture of DK completion flags
                    dk_done_latch <= dk_done_latch or dk_done_array;
                    
                    if unsigned(not (dk_done_latch or dk_done_array)) = 0 then
                        state <= CALC_ERR_JAC;
                    end if;

                when CALC_ERR_JAC =>
                    -- Extract Target Vectors (n, o, a)
                    n_tx := pos_target(0,0); n_ty := pos_target(1,0); n_tz := pos_target(2,0);
                    o_tx := pos_target(0,1); o_ty := pos_target(1,1); o_tz := pos_target(2,1);
                    a_tx := pos_target(0,2); a_ty := pos_target(1,2); a_tz := pos_target(2,2);
                    
                    -- Extract Current Real Vectors
                    n_cx := t_mat_real(0,0)(TOTAL_WIDTH-1 downto 0);
                    n_cy := t_mat_real(1,0)(TOTAL_WIDTH-1 downto 0);
                    n_cz := t_mat_real(2,0)(TOTAL_WIDTH-1 downto 0);
                    o_cx := t_mat_real(0,1)(TOTAL_WIDTH-1 downto 0);
                    o_cy := t_mat_real(1,1)(TOTAL_WIDTH-1 downto 0);
                    o_cz := t_mat_real(2,1)(TOTAL_WIDTH-1 downto 0);
                    a_cx := t_mat_real(0,2)(TOTAL_WIDTH-1 downto 0);
                    a_cy := t_mat_real(1,2)(TOTAL_WIDTH-1 downto 0);
                    a_cz := t_mat_real(2,2)(TOTAL_WIDTH-1 downto 0);

                    -- 3A. Spatial Error (XYZ)
                    temp_err_x := pos_target(0,3) - t_mat_real(0,3)(TOTAL_WIDTH-1 downto 0);
                    temp_err_y := pos_target(1,3) - t_mat_real(1,3)(TOTAL_WIDTH-1 downto 0);
                    temp_err_z := pos_target(2,3) - t_mat_real(2,3)(TOTAL_WIDTH-1 downto 0);
                    
                    err_x <= temp_err_x; err_y <= temp_err_y; err_z <= temp_err_z;

                    -- 3B. Angular Error via Cross-Products
                    -- Avoids inverse trigonometry. Division by 2 via shift_right.
                    temp_err_rx := shift_right(
                        fp_mult(n_cy, n_tz) - fp_mult(n_cz, n_ty) +
                        fp_mult(o_cy, o_tz) - fp_mult(o_cz, o_ty) +
                        fp_mult(a_cy, a_tz) - fp_mult(a_cz, a_ty), 1);
                        
                    temp_err_ry := shift_right(
                        fp_mult(n_cz, n_tx) - fp_mult(n_cx, n_tz) +
                        fp_mult(o_cz, o_tx) - fp_mult(o_cx, o_tz) +
                        fp_mult(a_cz, a_tx) - fp_mult(a_cx, a_tz), 1);
                        
                    temp_err_rz := shift_right(
                        fp_mult(n_cx, n_ty) - fp_mult(n_cy, n_tx) +
                        fp_mult(o_cx, o_ty) - fp_mult(o_cy, o_tx) +
                        fp_mult(a_cx, a_ty) - fp_mult(a_cy, a_tx), 1);

                    err_rx <= temp_err_rx; err_ry <= temp_err_ry; err_rz <= temp_err_rz;

                    -- Total absolute error for convergence check
                    abs_err_total <= abs(temp_err_x) + abs(temp_err_y) + abs(temp_err_z) +
                                     abs(temp_err_rx) + abs(temp_err_ry) + abs(temp_err_rz);

                    -- 3C. Build the 6xN Jacobian Matrix
                    for i in 0 to DOF-1 loop
                        -- Linear part: Delta Position / DELTA_VAL. (Multiplied by 2^5)
                        jacobian(0, i) <= shift_left(t_mat_pert(i)(0,3)(TOTAL_WIDTH-1 downto 0) - t_mat_real(0,3)(TOTAL_WIDTH-1 downto 0), 5);
                        jacobian(1, i) <= shift_left(t_mat_pert(i)(1,3)(TOTAL_WIDTH-1 downto 0) - t_mat_real(1,3)(TOTAL_WIDTH-1 downto 0), 5);
                        jacobian(2, i) <= shift_left(t_mat_pert(i)(2,3)(TOTAL_WIDTH-1 downto 0) - t_mat_real(2,3)(TOTAL_WIDTH-1 downto 0), 5);

                        -- Extract perturbed vectors for joint 'i'
                        n_px := t_mat_pert(i)(0,0)(TOTAL_WIDTH-1 downto 0);
                        n_py := t_mat_pert(i)(1,0)(TOTAL_WIDTH-1 downto 0);
                        n_pz := t_mat_pert(i)(2,0)(TOTAL_WIDTH-1 downto 0);
                        o_px := t_mat_pert(i)(0,1)(TOTAL_WIDTH-1 downto 0);
                        o_py := t_mat_pert(i)(1,1)(TOTAL_WIDTH-1 downto 0);
                        o_pz := t_mat_pert(i)(2,1)(TOTAL_WIDTH-1 downto 0);
                        a_px := t_mat_pert(i)(0,2)(TOTAL_WIDTH-1 downto 0);
                        a_py := t_mat_pert(i)(1,2)(TOTAL_WIDTH-1 downto 0);
                        a_pz := t_mat_pert(i)(2,2)(TOTAL_WIDTH-1 downto 0);

                        -- Angular part: Cross-product between Real and Perturbed matrices
                        -- Divided by 2 and multiplied by 2^5 -> equals shift_left(..., 4)
                        jacobian(3, i) <= shift_left(
                            fp_mult(n_cy, n_pz) - fp_mult(n_cz, n_py) +
                            fp_mult(o_cy, o_pz) - fp_mult(o_cz, o_py) +
                            fp_mult(a_cy, a_pz) - fp_mult(a_cz, a_py), 4);
                            
                        jacobian(4, i) <= shift_left(
                            fp_mult(n_cz, n_px) - fp_mult(n_cx, n_pz) +
                            fp_mult(o_cz, o_px) - fp_mult(o_cx, o_pz) +
                            fp_mult(a_cz, a_px) - fp_mult(a_cx, a_pz), 4);

                        jacobian(5, i) <= shift_left(
                            fp_mult(n_cx, n_py) - fp_mult(n_cy, n_px) +
                            fp_mult(o_cx, o_py) - fp_mult(o_cy, o_px) +
                            fp_mult(a_cx, a_py) - fp_mult(a_cy, a_px), 4);
                    end loop;
                    
                    state <= THETA_NEXT;

                when THETA_NEXT =>
                    -- Convergence check
                    if abs_err_total < MAX_ERR or iter_count = MAX_ITER then
                        state <= FINISHED;
                    else
                        -- 4. UPDATE JOINT ANGLES (Gradient Descent Step)
                        for i in 0 to DOF-1 loop
                            -- Dot product of Jacobian Transpose column with 6D Error Vector
                            accum_theta := (jacobian(0, i) * err_x) + 
                                           (jacobian(1, i) * err_y) + 
                                           (jacobian(2, i) * err_z) +
                                           (jacobian(3, i) * err_rx) + 
                                           (jacobian(4, i) * err_ry) + 
                                           (jacobian(5, i) * err_rz);
                                           
                            -- Alpha learning rate scaling. 
                            -- Divides the step size by 32 (FRAC_WIDTH + 6) for stability.
                            theta_curr(i) <= theta_curr(i) + resize(shift_right(accum_theta, FRAC_WIDTH + 6), TOTAL_WIDTH);
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