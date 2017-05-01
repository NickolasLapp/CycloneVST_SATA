library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;

entity phy_layer_init is
    port(
        rxclkout         : in  std_logic;
        txclkout         : in  std_logic;
        reset            : in  std_logic;

        rx_ordered_data  : out std_logic_vector(31 downto 0);
        primitive_recvd  : out std_logic;

        rx_data          : in  std_logic_vector(31 downto 0);
        rx_datak         : in  std_logic_vector(3 downto 0);
        rx_signaldetect  : in  std_logic;

        rx_errdetect     : in std_logic_vector(3 downto 0);

        tx_forceelecidle : out std_logic;
        tx_data          : out std_logic_vector(31 downto 0);
        tx_datak         : out std_logic_vector(3 downto 0);

        do_word_align    : out std_logic;
        rx_syncstatus    : in std_logic_vector(3 downto 0);

        rx_set_locktodata: out std_logic;
        rx_set_locktoref : out std_logic;

        ppm_within_threshold:in std_logic;

        PHYRDY           : out std_logic
    );
end entity phy_layer_init;

architecture phy_layer_initt_arch of phy_layer_init is

    signal phy_init_state     : PHYINIT_STATE_TYPE := HP1_HR_RESET;
    signal phy_init_next_state : PHYINIT_STATE_TYPE := HP1_HR_RESET;
    signal phy_init_prev_state : PHYINIT_STATE_TYPE := HP1_HR_RESET;

    signal resume_pending : std_logic := '0';

    signal oob_signal_to_send  : OOB_SIGNAL := NONE;
    signal oob_signal_received: OOB_SIGNAL := NONE;

    signal oob_rx_idle : std_logic;
    signal oob_tx_idle : std_logic;


    signal force_quiescent   : std_logic  := '0';
    signal force_active      : std_logic  := '0';
    signal tx_forceelecidle_oob_detector : std_logic := '0';

    signal retry_time_elapsed : std_logic_vector(31 downto 0) := (others => '0');

    signal consecutive_non_aligns : std_logic_vector(3 downto 0) := (others => '0');

    signal rx_ordered_data_s  : std_logic_vector(31 downto 0);
    signal primitive_recvd_s : std_logic;
    signal do_byte_order   : std_logic;
    signal is_byte_ordered : std_logic;

    signal signal_stable_time : std_logic_vector(31 downto 0) := (others => '0');
    signal stable_data     : std_logic_vector(31 downto 0) := (others => '0');
    signal stable_err_detect  : std_logic_vector(3 downto 0) := (others => '0');

    signal lock_done : std_logic := '0';

    component byte_orderer is
        port(
            rxclkout         : in  std_logic;
            reset            : in  std_logic;
            do_byte_order    : in  std_logic;

            rx_data          : in  std_logic_vector(31 downto 0);
            rx_datak         : in  std_logic_vector(3 downto 0);

            is_byte_ordered  : out std_logic;
            rx_ordered_data  : out std_logic_vector(31 downto 0);
            primitive_recvd  : out std_logic
        );
    end component byte_orderer;

    component OOB_signal_detect is
      port(
        rxclkout         : in  std_logic;
        txclkout         : in  std_logic;
        reset            : in  std_logic;

        rx_data          : in  std_logic_vector(31 downto 0);
        rx_signaldetect  : in  std_logic;

        oob_signal_to_send  : in  OOB_SIGNAL;
        oob_rx_idle        : out std_logic;
        oob_tx_idle        : out std_logic;

        oob_signal_received: out OOB_SIGNAL;

        tx_forceelecidle : out std_logic
        );
    end component OOB_signal_detect;

    begin

    process(txclkout, reset)
    begin
        if(reset = '1') then
            tx_forceelecidle <= '0';
        elsif(rising_edge(txclkout)) then
            if(force_quiescent = '1' and oob_tx_idle = '1') then
                tx_forceelecidle <= '1';
            elsif(force_active = '1') then
                tx_forceelecidle <= '0';
            else
                tx_forceelecidle <= tx_forceelecidle_oob_detector;
            end if;
        end if;
    end process;

    byte_order1 : byte_orderer
        port map(
            rxclkout         => rxclkout,
            reset            => reset,
            do_byte_order    => do_byte_order,

            rx_data          => rx_data,
            rx_datak         => rx_datak,

            is_byte_ordered  => is_byte_ordered,
            rx_ordered_data  => rx_ordered_data_s,
            primitive_recvd  => primitive_recvd_s
        );

    process(rxclkout, reset)
    begin
        if(reset = '1') then
            rx_ordered_data <= (others => '0');
            primitive_recvd <= '0';
            consecutive_non_aligns <= (others => '0');
        elsif(rising_edge(rxclkout)) then
            rx_ordered_data <= rx_ordered_data_s;
            primitive_recvd <= primitive_recvd_s;
            if(rx_ordered_data_s(7 downto 0) = DATAK_28_3) then
                consecutive_non_aligns <= consecutive_non_aligns + 1;
            else
                consecutive_non_aligns <= (others => '0');
            end if;
        end if;
    end process;

    signal_detect1 : OOB_signal_detect
        port map(
            rxclkout            => rxclkout,
            txclkout            => txclkout,
            reset               => reset,

            rx_data             => rx_data,
            rx_signaldetect     => rx_signaldetect,

            oob_signal_to_send     => oob_signal_to_send,
            oob_rx_idle           => oob_rx_idle,
            oob_tx_idle           => oob_tx_idle,

            oob_signal_received   => oob_signal_received,
            tx_forceelecidle    => tx_forceelecidle_oob_detector
        );

    -- Retry_interval Counter
    process(txclkout, reset)
    begin
        if(reset = '1') then
            retry_time_elapsed <= (others => '0');
        elsif(rising_edge(txclkout)) then
            if(phy_init_state = phy_init_prev_state) then
                retry_time_elapsed <= retry_time_elapsed + 1;
            else
                retry_time_elapsed <= (others => '0');
            end if;
        end if;
    end process;



    -- State Register
    process(txclkout, reset)
    begin
        if(reset = '1') then
            phy_init_state <= HP1_HR_RESET;
            phy_init_prev_state <= HP1_HR_RESET;
        elsif(rising_edge(txclkout)) then
            phy_init_state <= phy_init_next_state;
            phy_init_prev_state <= phy_init_state;
        end if;
    end process;

    -- Output Logic
    process(txclkout, reset)
    begin
        if(reset = '1') then
            force_quiescent <= '1';
            force_active    <= '0';
            oob_signal_to_send <= NONE;
            tx_data <= (others => '0');
            PHYRDY <= '0';
            rx_set_locktodata <= '0';
            rx_set_locktoref  <= '0';
            do_word_align <= '0';
            do_byte_order <= '0';
            resume_pending <= '0';
            signal_stable_time <= (others => '0');
            stable_data       <= (others => '0');
            stable_err_detect  <= (others => '0');
            lock_done <= '0';
        elsif(rising_edge(txclkout)) then
            case phy_init_state is
                when    HP1_HR_Reset                =>
                -- transmit COMRESET
                    rx_set_locktodata <= '0';
                    rx_set_locktoref  <= '1';
                    force_quiescent <= '0';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= COMRESET;
                    do_word_align <= '0';
                    do_byte_order <= '0';
                    resume_pending <= '0';
                    signal_stable_time <= (others => '0');
                    stable_data     <= (others => '0');
                    stable_err_detect  <= (others => '0');
                    lock_done <= '0';
                when    HP2_HR_Await_COMINIT         =>
                -- Quiescent
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP2_B_HR_Await_No_COMINIT      =>
                --Quiescent
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP3_HR_Calibrate            =>
                -- Sending out Calibration Signals
                -- NOT SUPPORTED FOR NOW
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP4_HR_COMWAKE              =>
                -- Transmit COMWAKE
                    force_quiescent <= '0';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    if(oob_tx_idle = '1') then
                        oob_signal_to_send <= COMWAKE;
                    else
                        oob_signal_to_send <= NONE;
                    end if;
                when    HP5_HR_Await_COMWAKE         =>
                -- Quiescent
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP5_B_HR_Await_No_COMWAKE      =>
                -- Quiescent
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP6_HR_Await_Align           =>
                -- Transmit d10.2 words! (tx_data <= 0h'4A4A4A4A')???
                    force_quiescent <= '0';
                    force_active    <= '1';
                    tx_data <= x"4A4A4A4A";
                    tx_datak <= DATAK_BYTE_NONE;
                    oob_signal_to_send <= NONE;

                    if(stable_data = rx_data) then-- and stable_err_detect = rx_errdetect) then
                        signal_stable_time <= signal_stable_time + '1';

                        if(signal_stable_time > 15 and rx_syncstatus = ALL_WORDS_SYNC) then
                            do_byte_order <= '1';

                            if(is_byte_ordered = '1' and rx_ordered_data_s = ALIGNp and lock_done = '0') then
                                lock_done <= '1';
                                rx_set_locktoref <= '0';
                                rx_set_locktodata <= '1';
                                do_byte_order <= '0';
                                signal_stable_time <= (others => '0');
                                stable_data <= (others => '0');
                                stable_err_detect <= (others => '1');
                            else
                                do_word_align <= retry_time_elapsed(6);
                            end if;
                        else
                            do_word_align <= retry_time_elapsed(6);
                            do_byte_order <= '0';
                        end if;
                    else
                        signal_stable_time <= (others => '0');
                        stable_data       <= rx_data;
                        stable_err_detect  <= rx_errdetect;
                        do_word_align <= retry_time_elapsed(6);
                        do_byte_order <= '0';
                    end if;

                when    HP7_HR_Send_Align            =>
                -- Transmit ALIGNp
                    force_quiescent <= '0';
                    force_active    <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                    do_word_align <= '0';
                    do_byte_order <= '0';
                when    HP8_HR_Ready                =>
                -- tx_data <= tx_data (from link layer);
                    force_quiescent <= '0';
                    force_active    <= '1';
                    tx_data <= SYNCp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                    PHYRDY <= '1';
                when    HP9_HR_Partial              =>
                -- not supported
                    force_quiescent <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP10_HR_Slumber             =>
                -- not supported
                    force_quiescent <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP11_HR_Adjust_speed         =>
                -- not supported
                    force_quiescent <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    others                      =>
                -- not supported
                    force_quiescent <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                    PHYRDY <= '0';
            end case;
        end if;
    end process;

    -- Next State Logic
    process(phy_init_state, phy_init_prev_state, oob_signal_received, retry_time_elapsed, oob_rx_idle, oob_tx_idle, resume_pending, consecutive_non_aligns, is_byte_ordered, rx_syncstatus)
    begin
        case phy_init_state is
            when    HP1_HR_Reset                =>
                if(oob_tx_idle = '1') then
                    phy_init_next_state <= HP2_HR_AwaitCcOMINIT;
                else
                    phy_init_next_state <= HP1_HR_Reset;
                end if;

            when    HP2_HR_Await_COMINIT         =>
                if(oob_signal_received = COMINIT) then
                    phy_init_next_state <= HP2_B_HR_Await_No_COMINIT;
                elsif(retry_time_elapsed >= RETRY_INTERVAL and phy_init_state = phy_init_prev_state) then
                    phy_init_next_state <= HP1_HR_Reset;
                else
                    phy_init_next_state <= HP2_HR_Await_COMINIT;
                end if;

            when    HP2_b_HR_Await_No_COMINIT      =>
                if(oob_rx_idle = '1') then
                    phy_init_next_state <= HP4_HR_COMWAKE;
                else
                    phy_init_next_state <= HP2_B_HR_Await_No_cOMINIT;
                end if;

            when    HP3_HR_Calibrate            =>
            -- NOT SUPPORTED FOR NOW, should NEVER end up here since HP2_B bypasses this step
                phy_init_next_state <= HP4_HR_COMWAKE;

            when    HP4_HR_COMWAKE              =>
                if(oob_tx_idle = '0') then
                    phy_init_next_state <= HP4_HR_COMWAKE;
                elsif(oob_signal_received = COMWAKE) then
                    phy_init_next_state <= HP5_B_HR_Await_No_COMWAKE;
                else
                    phy_init_next_state <= HP5_HR_Await_COMWAKE;
                end if;

            when    HP5_HR_Await_COMWAKE         =>
                if(oob_signal_received = COMWAKE) then
                    phy_init_next_state <= HP5_B_HR_Await_No_COMWAKE;
                elsif(retry_time_elapsed > RETRY_INTERVAL and phy_init_state = phy_init_prev_state) then
                    if(resume_pending = '0') then
                        phy_init_next_state <= HP1_HR_RESET;
                    else
                        phy_init_next_state <= HP4_HR_COMWAKE;
                    end if;
                else
                    phy_init_next_state <= HP5_HR_Await_COMWAKE;
                end if;

            when    HP5_B_HR_Await_No_COMWAKE      =>
                if(oob_rx_idle = '1') then
                    phy_init_next_state <= HP6_HR_Await_Align;
                else
                    phy_init_next_state <= HP5_B_HR_Await_No_COMWAKE;
                end if;

            when    HP6_HR_Await_Align           =>
                if(rx_signaldetect = '0') then
                    phy_init_next_state <= HP4_HR_COMWAKE;
                elsif(rx_syncstatus = ALL_WORDS_SYNC and is_byte_ordered = '1' and rx_ordered_data_s = ALIGNp and lock_done = '1' and signal_stable_time > 32) then
                    phy_init_next_state <= HP7_HR_Send_Align;
                elsif(retry_time_elapsed > ALIGN_INTERVAL and phy_init_state = phy_init_prev_state) then
                    phy_init_next_state <= HP1_HR_RESET;
                else
                    phy_init_next_state <= HP6_HR_Await_Align;
                end if;

            when    HP7_HR_Send_Align            =>
                if(rx_signaldetect = '0') then
                    phy_init_next_state <= HP4_HR_COMWAKE;
                elsif(consecutive_non_aligns >= 3) then
                    phy_init_next_state <= HP8_HR_Ready;
                else
                    phy_init_next_state <= HP7_HR_Send_Align;
                end if;

            when    HP8_HR_Ready                =>
                if(rx_signaldetect = '0') then
                    phy_init_next_state <= HP1_HR_Reset;
                else
                    phy_init_next_state <= HP8_HR_Ready;
                end if;

            when    HP9_HR_Partial              =>
                -- currently not supported. Should never end here
                phy_init_next_state <= HP1_HR_Reset;
            when    HP10_HR_Slumber             =>
                -- currently not supported. Should never end here
                phy_init_next_state <= HP1_HR_Reset;
            when    HP11_HR_Adjust_speed         =>
                -- currently not supported. Should never end here
                phy_init_next_state <= HP1_HR_Reset;
            when    others                      =>
                -- currently not supported. Should never end here
                phy_init_next_state <= HP1_HR_Reset;
        end case;
    end process;
end architecture Phy_layer_init_arch;
