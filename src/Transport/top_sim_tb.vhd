library ieee;                   --! Use standard library.
use ieee.std_logic_1164.all;    --! Use standard logic elements
use ieee.numeric_std.all;       --! Use numeric standard

use work.transport_layer_pkg.all;

entity top_sim_tb is
end top_sim_tb;

architecture top_sim_tb_arch of top_sim_tb is

	signal clk : std_logic := '0';
	signal rst_n : std_logic := '1';

	signal rx_serial_data_s, tx_serial_data_s, fake_button, fake_led : std_logic;

	--Interface with Link Layer
	signal status_to_link_tb : std_logic_vector(7 downto 0); --for test just use bit 0 to indicate data ready
	signal status_from_link_tb : std_logic_vector(7 downto 0);
	signal data_to_link_tb : std_logic_vector(DATA_WIDTH - 1 downto 0);
	signal data_from_link_tb : std_logic_vector(DATA_WIDTH - 1 downto 0);


	signal trans_rx_from_link_ready, trans_tx_to_link_ready : std_logic;

	constant clk_period : time := 1 ns; --not accurate to device

	component top_sim is
    	port(
	        clk50 : in std_logic;           -- 50 MHz clock from AC18, driven by SL18860C
	        cpu_rst_n : in std_logic;       -- CPU_RESETn pushbutton. (Debounce this). Pin AD27

	        pll_refclk_150 : in std_logic;  -- 150MHz PLL refclk for XCVR design,
	                                        -- driven by Si570 (need to change clock frequency with Clock Control GUI)
	        rx_serial_data : in  std_logic; -- XCVR input serial line.
	        tx_serial_data : out std_logic; -- XCVR output serial line

	        USER_PB_FPGA1  : in  std_logic; -- PB1, used as alternative reset so that we can reset without resetting the clock control circuits
	        USER_LED_FPGA0 : out std_logic;  -- LED0 for heartbeat

	        --Interface with Link Layer
	        status_to_link_top :    out std_logic_vector(7 downto 0); --for test just use bit 0 to indicate data ready
	        status_from_link_top     :   in std_logic_vector(7 downto 0);
	        data_to_link_top     :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
	        data_from_link_top      :   in std_logic_vector(DATA_WIDTH - 1 downto 0)
        );
	end component top_sim;


begin
	dut : top_sim
		port map(
			clk50 => clk,
			cpu_rst_n => rst_n,
			pll_refclk_150 => clk,
			rx_serial_data => rx_serial_data_s,
			tx_serial_data => tx_serial_data_s,
			USER_PB_FPGA1 => fake_button,
			USER_LED_FPGA0 => fake_led,
			status_to_link_top => status_to_link_tb,
			status_from_link_top => status_from_link_tb,
			data_to_link_top => data_to_link_tb,
			data_from_link_top => data_from_link_tb
		);

	clk_process	: process
	begin
		clk <= '0';
		wait for clk_period/2;
		clk <= '1';
		wait for clk_period/2;
	end process;

	stim_proc : process
		begin
		wait for 8 ns;
		rst_n <= '0'; wait for 10 ns;
		rst_n <= '1'; wait until rising_edge(clk);
		status_from_link_tb <= "10111111"; wait until rising_edge(clk);
		data_from_link_tb <= x"000000" & REG_DEVICE_TO_HOST; wait until rising_edge(clk);
		for i in 0 to 3 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "00111111"; wait until rising_edge(clk);
		wait until data_to_link_tb = x"00EC8027";
		for i in 0 to 5 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "10111111"; wait until rising_edge(clk);
		data_from_link_tb <= x"000000" & PIO_SETUP_FIS; wait until rising_edge(clk);
		for i in 0 to 5 loop wait until rising_edge(clk); end loop;
		data_from_link_tb <= x"000000" & DATA_FIS; wait until rising_edge(clk);
		for i in 0 to 5 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "00111111"; wait until rising_edge(clk);
		wait until data_to_link_tb = x"00358027";

		for i in 0 to 11 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "10111111"; wait until rising_edge(clk);
		data_from_link_tb <= x"00000039"; wait until rising_edge(clk);
		data_from_link_tb <= x"FFFFFFFF"; wait until rising_edge(clk);
		status_from_link_tb <= "00111111"; wait until rising_edge(clk);
		for i in 0 to 3 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "01111111"; wait until rising_edge(clk);--pause
		--wait until rising_edge(clk);
		status_from_link_tb <= "00111111"; wait until rising_edge(clk);

		wait until trans_tx_to_link_ready = '0'; wait until rising_edge(clk);
		status_from_link_tb <= "00011111"; wait until rising_edge(clk);

		for i in 0 to 21 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "10011111"; wait until rising_edge(clk);
		data_from_link_tb <= x"00000034"; wait until rising_edge(clk);
		for i in 0 to 3 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "00011111"; wait until rising_edge(clk);
		wait until trans_tx_to_link_ready = '1'; wait until rising_edge(clk);
		status_from_link_tb <= "00111111"; wait until rising_edge(clk);
		wait until trans_tx_to_link_ready = '0'; wait until rising_edge(clk);

		for i in 0 to 23 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "10011111"; wait until rising_edge(clk);
		data_from_link_tb <= x"00000046"; wait until rising_edge(clk);
		for i in 0 to BUFFER_DEPTH - 1 loop
			data_from_link_tb <= std_logic_vector(to_unsigned(i,32)); wait until rising_edge(clk);
		end loop;
		status_from_link_tb <= "00011111"; wait until rising_edge(clk);
		for i in 0 to 21 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "10011111"; wait until rising_edge(clk);
		data_from_link_tb <= x"00000034"; wait until rising_edge(clk);
		for i in 0 to 3 loop wait until rising_edge(clk); end loop;
		status_from_link_tb <= "00011111"; wait until rising_edge(clk);
		for i in 0 to BUFFER_DEPTH*10 loop wait until rising_edge(clk); end loop;
	end process;

	trans_rx_from_link_ready <= status_to_link_tb(6);
	trans_tx_to_link_ready <= status_to_link_tb(5);
end architecture;