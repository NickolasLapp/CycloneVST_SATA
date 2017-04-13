library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;
use work.transport_layer_pkg.all;

entity top_sim is
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
end top_sim;

architecture top_sim_arch of top_sim is
    -- top level signals
    signal reset            : std_logic;
    signal cpu_rst          : std_logic;
    signal cpu_rst_debounced: std_logic;
    signal pb_fpga1         : std_logic;

    signal fabric_clk_37_5  : std_logic; -- use this to clock the receive datapath
    -- xcvr signals
    --PHY Control Signals
    signal rxclkout                : std_logic; -- use this to clock the receive datapath
    signal txclkout                : std_logic; -- use this to clock the transmit datapath
    signal pll_locked               : std_logic; -- is the pll reference clock locked in

    signal rx_data         : std_logic_vector(31 downto 0); -- received data
    signal tx_data         : std_logic_vector(31 downto 0); -- data to transmit

    signal tx_forceelecidle         : std_logic; -- force signal idle for OOB signaling
    signal rx_signaldetect          : std_logic; -- detect signal idle for OOB signaling
    signal rx_is_lockedtoref        : std_logic; -- receiver is locked to pll reference clock
    signal rx_is_lockedtodata       : std_logic; -- receiver is locked to CDR block from received data


    signal rx_pma_clkout            : std_logic; -- recovered clock from cdr circuitry.

    signal do_word_align            : std_logic; -- perform word alignment to the comma 28.5 character
    signal rx_patterndetect         : std_logic_vector(3 downto 0); -- are we detecting the comma character in received data?
    signal rx_syncstatus            : std_logic_vector(3 downto 0); -- are we synced to data? (why is this 4 bits wide...?)

    signal rx_errdetect             : std_logic_vector(3 downto 0); -- reports a 8B/10B Code violation
    signal rx_disperr               : std_logic_vector(3 downto 0); -- reports a 8B/10B Disparity Error

    signal tx_datak                 : std_logic_vector(3 downto 0); -- send control character on specified byte instead of data character (k28.5 instead of d28.5)
    signal rx_datak                 : std_logic_vector(3 downto 0); -- reports which bytes contained control characters

    signal tx_ready           : std_logic; -- is the transmitter ready
    signal rx_ready           : std_logic; -- is the receiver ready

    --rst signals
    signal pll_powerdown      : std_logic; --      pll_powerdown.pll_powerdown
    signal tx_analogreset     : std_logic; --     tx_analogreset.tx_analogreset
    signal tx_digitalreset    : std_logic; --    tx_digitalreset.tx_digitalreset
    signal rx_analogreset     : std_logic; --     rx_analogreset.rx_analogreset
    signal rx_digitalreset    : std_logic; --    rx_digitalreset.rx_digitalreset

    signal tx_cal_busy        : std_logic; --        tx_cal_busy.tx_cal_busy
    signal rx_cal_busy        : std_logic; --        rx_cal_busy.rx_cal_busy

    signal rx_set_locktodata  : std_logic;
    signal rx_set_locktoref   : std_logic;

    -- reconfig signals
    signal reconfig_from_xcvr       : std_logic_vector(91 downto 0);
    signal reconfig_to_xcvr         : std_logic_vector(139 downto 0);
    signal reconfig_busy            : std_logic;

    -- link/phy hookup signals
    signal phy_status_to_link       : std_logic_vector(PHY_STATUS_LENGTH-1 downto 0);
    signal link_status_to_phy       : std_logic_vector(LINK_STATUS_LENGTH-1 downto 0);
    signal tx_data_from_link        : std_logic_vector(31 downto 0);
    signal rx_data_to_link          : std_logic_vector(31 downto 0);

    signal trans_status_in          : std_logic_vector(7 downto 0);
    signal trans_status_out         : std_logic_vector(7 downto 0);
    signal trans_tx_data_in         : std_logic_vector(31 downto 0);
    signal trans_rx_data_out        : std_logic_vector(31 downto 0);
    signal rst_n                    : std_logic;

     --signal declarations for dummy application process
    signal user_cmd_to_trans : std_logic_vector(2 downto 0);
    signal user_data_to_trans : std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal user_address_to_trans : std_logic_vector(DATA_WIDTH - 1 downto 0);

    signal trans_status_to_user : std_logic_vector(3 downto 0);
    signal trans_data_to_user : std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal trans_address_to_user : std_logic_vector(DATA_WIDTH - 1 downto 0);

    signal app_control_counter : integer range 0 to 1000001;
    signal app_data_counter : integer range 0 to BUFFER_DEPTH;

    signal logical_sector_index : integer range 0 to BUFFER_DEPTH;
    signal app_count_up : std_logic;

    signal msata_device_ready : std_logic;
    signal app_write_valid : std_logic;
    signal app_send_read_valid : std_logic;
    signal app_receive_read_valid : std_logic;
    signal test_write_address : std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal app_read_sent : std_logic;
    signal error_counter : integer range 0 to integer'high;
    signal words_read    : integer range 0 to integer'high;

    signal user_cmd_to_trans_prev : std_logic_vector(2 downto 0);
    signal app_receive_read_valid_prev : std_logic;

    component transport_layer is
        port(
            --Interface with Application Layer
            rst_n           :   in std_logic;
            clk         :   in std_logic;

            data_from_user      :   in std_logic_vector(DATA_WIDTH - 1 downto 0);
            address_from_user   :   in std_logic_vector(DATA_WIDTH - 1 downto 0);

            user_command            :   in std_logic_vector(2 downto 0);
            status_to_user          :   out std_logic_vector(3 downto 0);

            data_to_user       :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
            address_to_user    :   out std_logic_vector(DATA_WIDTH - 1 downto 0);

            --Interface with Link Layer
            status_to_link :    out std_logic_vector(7 downto 0); --for test just use bit 0 to indicate data ready
            status_from_link     :   in std_logic_vector(7 downto 0);
            data_to_link     :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
            data_from_link      :   in std_logic_vector(DATA_WIDTH - 1 downto 0));

    end component transport_layer;





    component Debounce is
      port(
        clk50      : in  std_logic;
        button     : in  std_logic;
        debounced  : out std_logic);
    end component Debounce;

    begin

    --i_transdummy1 : transport_dummy
    --port map(
    --        fabric_clk => txclkout,
    --        reset      => rst_n,
    --        trans_status_to_link => trans_status_in,
    --        link_status_to_trans => trans_status_out,
    --        tx_data_to_link      => trans_tx_data_in,
    --        rx_data_from_link    => trans_rx_data_out
    --    );
    i_transport_layer1 : transport_layer
        port map(


            clk => clk50,
            rst_n      => cpu_rst_n,

            --Interface with Application Layer
            data_from_user => user_data_to_trans,
            address_from_user => user_address_to_trans,

            user_command => user_cmd_to_trans,
            status_to_user => trans_status_to_user,

            data_to_user => trans_data_to_user,
            address_to_user => trans_address_to_user,

            --Interface with Link Layer
            status_to_link => trans_status_in,

            status_from_link => trans_status_out,
            data_to_link => trans_tx_data_in,
            data_from_link => trans_rx_data_out
            );

    --Interface with Link Layer
    status_to_link_top <= trans_status_in;
    trans_status_out <= status_from_link_top;
    data_to_link_top <= trans_tx_data_in;
    trans_rx_data_out <= data_from_link_top;

--    i_linkLayer1 : link_layer_32bit
--    port map(   -- Input
--            clk             => clk50,
--            rst_n           => rst_n,

--            --Interface with Transport Layer
--            trans_status_in => trans_status_in,
--            trans_status_out=> trans_status_out,
--            tx_data_in      => trans_tx_data_in,
--            rx_data_out     => trans_rx_data_out,

--            --Interface with Physical Layer
--            tx_data_out     => tx_data_from_link,
--            rx_data_in      => rx_data_to_link,
--            phy_status_in   => phy_status_to_link,
--            phy_status_out  => link_status_to_phy
----            perform_init    => perform_init
--        );

    -- debounce reset and pushbutton.
    cpu_rst <= not cpu_rst_n;
    resetDebounce_0 : Debounce
        port map(clk50, cpu_rst, cpu_rst_debounced);

    pb_debounce : Debounce
        port map(clk50, USER_PB_FPGA1, pb_fpga1);

    reset <= (not pb_fpga1) or cpu_rst_debounced;
    rst_n <= not reset;
    USER_LED_FPGA0 <= '1' when pb_fpga1 = '1' else '0';

    -- dummy status and data values
--    link_status_to_phy <= LINK_STATUS_DEFAULT(LINK_STATUS_LENGTH-1 downto 0);
--    tx_data_from_link  <= SYNCp;

--dummy process to act as user application
    user_application  :   process(clk50, cpu_rst_n)
    begin
        if(cpu_rst_n = '0')then
            user_cmd_to_trans <= "000";
            user_data_to_trans <= (others => '0');
            user_address_to_trans <= (others => '0');
            app_data_counter <= 0;
            app_control_counter <= 0;
            app_read_sent <= '0';
            logical_sector_index <= 0;
            app_count_up <= '1';
            error_counter <= 0;
            words_read <= 0;
            user_cmd_to_trans_prev <= "000";
            app_receive_read_valid_prev <= '0';
        elsif(rising_edge(clk50))then
            user_cmd_to_trans_prev <= user_cmd_to_trans;
            app_receive_read_valid_prev <= app_receive_read_valid;
            if(msata_device_ready = '1')then
                if(app_control_counter < (2 * 2 * BUFFER_DEPTH))then --send write
                    if(app_write_valid = '1')then
                        if(app_data_counter < BUFFER_DEPTH)then
                            user_cmd_to_trans <= "001";--send write
                            --user_address_to_trans <= test_write_address;
                            user_address_to_trans <= std_logic_vector(to_unsigned(logical_sector_index,DATA_WIDTH));
                            if(app_count_up = '1')then
                                user_data_to_trans <= std_logic_vector(to_unsigned(app_data_counter,DATA_WIDTH));
                            else
                                user_data_to_trans <= std_logic_vector(to_unsigned(BUFFER_DEPTH - 1 - app_data_counter,DATA_WIDTH));
                            end if;
                            app_data_counter <= app_data_counter + 1;
                        else
                            user_cmd_to_trans <= "000";
                            user_data_to_trans <= (others => '1');
                            user_address_to_trans <= (others => '1');
                        end if;
                        app_control_counter <= app_control_counter + 1;
                    else
                        app_control_counter <= app_control_counter;
                    end if;
                elsif(app_control_counter < 4 * 2 * BUFFER_DEPTH)then --send read
                    app_data_counter <= 0;
                    if(app_send_read_valid = '1' and app_read_sent = '0')then
                        user_cmd_to_trans <= "010";--send read
                        user_address_to_trans <= std_logic_vector(to_unsigned(logical_sector_index,DATA_WIDTH));
                        user_data_to_trans <= (others => '1');
                        app_read_sent <= '1';
                    elsif(app_read_sent = '1')then
                        user_cmd_to_trans <= "000";
                        if(app_write_valid = '1' or app_send_read_valid = '1')then
                            app_control_counter <= app_control_counter + 1;
                        end if;
                    else
                        app_control_counter <= app_control_counter;
                    end if;
                elsif(app_control_counter < 6 * 2 * BUFFER_DEPTH)then --retrieve read
                    if(app_receive_read_valid = '1')then
                        user_cmd_to_trans <= "100";
                        user_address_to_trans <= std_logic_vector(to_unsigned(logical_sector_index,DATA_WIDTH));
                        user_data_to_trans <= (others => '1');
                    else
                        user_cmd_to_trans <= "000";
                    end if;

                    if(user_cmd_to_trans_prev = "100" and app_receive_read_valid_prev = '1') then
                        if(app_data_counter < BUFFER_DEPTH)then
                            app_data_counter <= app_data_counter + 1;
                        end if;
                        --something <= trans_data_to_user;
                        if (app_count_up = '1') then
                            if(trans_data_to_user /= std_logic_vector(to_unsigned(app_data_counter,DATA_WIDTH))) then
                                error_counter <= error_counter + 1;
                            end if;
                        else -- down
                            if(trans_data_to_user /= std_logic_vector(to_unsigned(BUFFER_DEPTH - 1 - app_data_counter,DATA_WIDTH))) then
                                error_counter <= error_counter + 1;
                            end if;
                        end if;
                        words_read <= words_read + 1;
                    end if;

                    app_control_counter <= app_control_counter + 1;
                elsif(app_control_counter >= 8 * 2 * BUFFER_DEPTH)then --reset
                    user_cmd_to_trans <= "000";
                    user_data_to_trans <= (others => '0');
                    user_address_to_trans <= (others => '0');
                    if(logical_sector_index < BUFFER_DEPTH)then
                        logical_sector_index <= logical_sector_index + BUFFER_DEPTH/DWORDS_PER_SECTOR;
                    else
                        logical_sector_index <= 0;
                        app_count_up <= not app_count_up;
                    end if;
                    app_control_counter <= 0;
                    app_data_counter <= 0;
                    app_read_sent <= '0';
                else --wait and increment
                    user_cmd_to_trans <= "000";
                    user_data_to_trans <= (others => '0');
                    user_address_to_trans <= (others => '0');
                    app_control_counter <= app_control_counter + 1;
                end if;
            end if;
        end if;
    end process;

    msata_device_ready <= trans_status_to_user(0);
    app_write_valid <= trans_status_to_user(1);
    app_send_read_valid <= trans_status_to_user(2);
    app_receive_read_valid <= trans_status_to_user(3);

    test_write_address <= (others  => '0'); --remove this to allow address functionaity

end top_sim_arch;
