library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;

entity phy_layer_init is
    port(
        rxclkout         : in  std_logic;
        txclkout         : in  std_logic;
        rst_n            : in  std_logic;

        rx_ordered_data  : out std_logic_vector(31 downto 0); -- the ordered data, after byte order completed
        primitive_recvd  : out std_logic; -- are we receiving a primitive (corresponds to ORDERED data!)

        rx_data          : in  std_logic_vector(31 downto 0); -- raw data received
        rx_datak         : in  std_logic_vector(3 downto 0); -- raw datak received
        rx_signaldetect  : in  std_logic; -- is the link active?

        rx_errdetect     : in std_logic_vector(3 downto 0); -- is there an error on the incoming data? not currently used, but could make the locking more roboust

        tx_forceelecidle : out std_logic; -- force the link idle? This is the signal actually used. Will get a different input depending on if we are sending an oob signal, or data.
        tx_data          : out std_logic_vector(31 downto 0); -- raw data to transmit.
        tx_datak         : out std_logic_vector(3 downto 0);

        do_word_align    : out std_logic; -- should we tell the XCVRs to align? This uses manual alignment
        rx_syncstatus    : in std_logic_vector(3 downto 0); -- are the XCVRs synced to ALIGNp? (should happen after do_word_align goes high if the device is sending ALIGNp at correct speed)

        rx_set_locktodata: out std_logic; -- how do the XCVRs locK? Needs to switch from locktoref -> locktodata when we know we are receiving aligns
        rx_set_locktoref : out std_logic;

        PHYRDY           : out std_logic
    );
end entity phy_layer_init;

architecture phy_layer_init_arch of phy_layer_init is

    signal phy_init_state     : PHYINIT_STATE_TYPE := HP1_HR_RESET;
    signal phy_init_next_state : PHYINIT_STATE_TYPE := HP1_HR_RESET;
    signal phy_init_prev_state : PHYINIT_STATE_TYPE := HP1_HR_RESET;

    signal resume_pending : std_logic := '0'; -- not currently used. Would be used if powering up from low power state

    signal oob_signal_to_send  : OOB_SIGNAL := NONE; -- what oob signal do we need to send? Passed to oobsignaldetect block
    signal oob_signal_received: OOB_SIGNAL := NONE; -- what oob signal are we recving? From oobsignaldetect block

    signal oob_rx_idle : std_logic; -- is the oobSignalDetector receiving a signal?
    signal oob_tx_idle : std_logic; -- is the oobSignalDetector done sending a signal?


    -- these signals determine whether we should send tx_forceelecidle high. When we are sending an oob siganl, we let the oobsignaldetector be in control.
    -- If we are sending data (i.e. D10.2s to align the stream) we have to force it low (force_active='1').
    -- Otherwise we force it high if the state machine requires it is high
    signal force_quiescent   : std_logic  := '0';
    signal force_active      : std_logic  := '0';
    signal tx_forceelecidle_oob_detector : std_logic := '0';


    -- counter to determine if we should reset the init sequence
    signal retry_time_elapsed : std_logic_vector(31 downto 0) := (others => '0');

    -- at the end of init, look for primitives that are not ALGINp. This tells us the data is good to pass to link
    signal consecutive_non_aligns : std_logic_vector(3 downto 0) := (others => '0');


    signal rx_ordered_data_s  : std_logic_vector(31 downto 0);    -- ordered data from byte orderer
    signal primitive_recvd_s : std_logic; -- is the ordered data primitive?
    signal do_byte_order   : std_logic; --indicate that word alignment good, perform byte order
    signal is_byte_ordered : std_logic; --success/fail of byte orderer

    signal signal_stable_time : std_logic_vector(31 downto 0) := (others => '0'); --during locking, before we switch clocks, we need the data to stabilize to make sure we are receiving at the correct data rate
    signal stable_data     : std_logic_vector(31 downto 0) := (others => '0'); --during locking, before we switch clocks, we need the data to stabilize to make sure we are receiving at the correct data rate
    signal stable_err_detect  : std_logic_vector(3 downto 0) := (others => '0'); --during locking, before we switch clocks, we need the data to stabilize to make sure we are receiving at the correct data rate

    signal lock_done : std_logic := '0'; -- did we switch the pll reference from refclk to data?

    component byte_orderer is
        port(
            rxclkout         : in  std_logic;
            rst_n            : in  std_logic;
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
        rst_n            : in  std_logic;

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

    process(txclkout, rst_n)
    begin
        if(rst_n = '0') then
            tx_forceelecidle <= '0';
        elsif(rising_edge(txclkout)) then
            if(force_quiescent = '1' and oob_tx_idle = '1') then -- force Quiescent takes priority, unless the oob signal detector is not finished sending its signal
                tx_forceelecidle <= '1';
            elsif(force_active = '1') then -- otherwise forceactive takes priority
                tx_forceelecidle <= '0';
            else -- default is the oob block
                tx_forceelecidle <= tx_forceelecidle_oob_detector;
            end if;
        end if;
    end process;

    byte_order1 : byte_orderer
        port map(
            rxclkout         => rxclkout,
            rst_n            => rst_n,
            do_byte_order    => do_byte_order,

            rx_data          => rx_data,
            rx_datak         => rx_datak,

            is_byte_ordered  => is_byte_ordered,
            rx_ordered_data  => rx_ordered_data_s,
            primitive_recvd  => primitive_recvd_s
        );


    -- count the number of non aligns we have received. Also buffer the rx_ordered_data
    process(rxclkout, rst_n)
    begin
        if(rst_n = '0') then
            rx_ordered_data <= (others => '0');
            primitive_recvd <= '0';
            consecutive_non_aligns <= (others => '0');
        elsif(rising_edge(rxclkout)) then
            rx_ordered_data <= rx_ordered_data_s;
            primitive_recvd <= primitive_recvd_s;
            if(rx_ordered_data_s(7 downto 0) = DATAK_28_3) then -- it is enough to just check the last byte to make sure it is 28_3 (not 28_5 like for align) to look for non-aligns
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
            rst_n               => rst_n,

            rx_data             => rx_data,
            rx_signaldetect     => rx_signaldetect,

            oob_signal_to_send     => oob_signal_to_send,
            oob_rx_idle           => oob_rx_idle,
            oob_tx_idle           => oob_tx_idle,

            oob_signal_received   => oob_signal_received,
            tx_forceelecidle    => tx_forceelecidle_oob_detector
        );

    -- Retry_interval Counter. Determines when we should start over our init process
    process(txclkout, rst_n)
    begin
        if(rst_n = '0') then
            retry_time_elapsed <= (others => '0');
        elsif(rising_edge(txclkout)) then
            if(phy_init_state = phy_init_prev_state) then --if we are in the same state, count up
                retry_time_elapsed <= retry_time_elapsed + 1;
            else --else reset
                retry_time_elapsed <= (others => '0');
            end if;
        end if;
    end process;



    -- State Register
    process(txclkout, rst_n)
    begin
        if(rst_n = '0') then
            phy_init_state <= HP1_HR_RESET;
            phy_init_prev_state <= HP1_HR_RESET;
        elsif(rising_edge(txclkout)) then
            phy_init_state <= phy_init_next_state;
            phy_init_prev_state <= phy_init_state;
        end if;
    end process;

    -- Output Logic
    process(txclkout, rst_n)
    begin
        if(rst_n = '0') then
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
                when    HP1_HR_Reset                => -- everything is reset in HP_RESET. we send a reset
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
                    force_quiescent <= '1';-- link must be quiescent, unless we are still sending COMRESET
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP2B_HR_Await_No_COMINIT      => -- wait until we are no longer receiving comint before continuting
                --Quiescent
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP3_HR_Calibrate            => -- skips over this state
                -- Sending out Calibration Signals
                -- NOT SUPPORTED FOR NOW
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP4_HR_COMWAKE              => -- send a comwake to the device
                -- Transmit COMWAKE
                    force_quiescent <= '0';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    if(oob_tx_idle = '1') then -- if we haven't started sending it yet, send a signal
                        oob_signal_to_send <= COMWAKE;
                    else
                        oob_signal_to_send <= NONE; -- otherwise tell the siganl sender to be idle after the first one
                    end if;
                when    HP5_HR_Await_COMWAKE         => -- wait to receive a comwake
                -- Quiescent
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP5B_HR_Await_No_COMWAKE      => --wait until oob idle
                -- Quiescent
                    force_quiescent <= '1';
                    force_active    <= '0';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP6_HR_Await_Align           => -- transmit d10.2 words while waiting for align
                -- Transmit d10.2 words! (tx_data <= 0h'4A4A4A4A')
                    force_quiescent <= '0';
                    force_active    <= '1';
                    tx_data <= x"4A4A4A4A"; -- d10.2
                    tx_datak <= DATAK_BYTE_NONE; -- not primitives!
                    oob_signal_to_send <= NONE;


                    -- The device will start at 6.0 Gbps, then 3.0 Gbps, the 1.5 Gbps. We can't immediatel lock to the data therefore,
                    -- because the data will not match the specified speed we are looking for. Therefore we wait until the data stabilizes,
                    -- which indicates that we are receiving data at the correct speed. Once this happens, we can switch our locks and resynch.
                    if(stable_data = rx_data) then-- looks like we are receiving stable data
                        signal_stable_time <= signal_stable_time + '1'; -- check how long we have received stable data

                        if(signal_stable_time > 15 and rx_syncstatus = ALL_WORDS_SYNC) then -- reasonable amount of stability, and the word aligner achieved success matching ALIGNp!
                            do_byte_order <= '1'; -- start the byte orderer

                            if(is_byte_ordered = '1' and rx_ordered_data_s = ALIGNp and lock_done = '0') then -- this is our first lock. At this point we have been receiving good data
                                                                                                              -- for a while and the word aligner was successful and the byte orderer was successful.
                                                                                                              -- we should be okay to switch the reference clocks.
                                lock_done <= '1'; -- we have switch the pll reference
                                rx_set_locktoref <= '0'; -- stop locking to refclk
                                rx_set_locktodata <= '1'; -- start locking to data
                                do_byte_order <= '0'; -- don't do byte order (will need to happen later again)
                                signal_stable_time <= (others => '0'); -- reset stable time (need to check this again after lock completes)
                                stable_data <= (others => '0'); -- reset stable time (need to check this again after lock completes)
                                stable_err_detect <= (others => '1'); -- reset stable time (need to check this again after lock completes)
                            else
                                do_word_align <= retry_time_elapsed(6); -- periodically try word alignment so we can see the ALIGNp come through
                            end if;
                        else -- periodically try word alignment so we can see the ALIGNp come through
                            do_word_align <= retry_time_elapsed(6);
                            do_byte_order <= '0';
                        end if;
                    else -- periodically try word alignment so we can see the ALIGNp come through
                        signal_stable_time <= (others => '0');
                        stable_data       <= rx_data;
                        stable_err_detect  <= rx_errdetect;
                        do_word_align <= retry_time_elapsed(6);
                        do_byte_order <= '0';
                    end if;

                when    HP7_HR_Send_Align            => -- send aligns, wait for the device to respond with different primitives
                -- Transmit ALIGNp
                    force_quiescent <= '0';
                    force_active    <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                    do_word_align <= '0';
                    do_byte_order <= '0';
                when    HP8_HR_Ready                => -- good to send!
                -- tx_data <= tx_data (from link layer);
                    force_quiescent <= '0';
                    force_active    <= '1';
                    tx_data <= SYNCp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                    PHYRDY <= '1';
                when    HP9_HR_Partial              => -- powerdown state. Not used
                -- not supported
                    force_quiescent <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP10_HR_Slumber             => -- powerdown state. Not used
                -- not supported
                    force_quiescent <= '1';
                    tx_data <= ALIGNp;
                    tx_datak <= DATAK_BYTE_ZERO;
                    oob_signal_to_send <= NONE;
                when    HP11_HR_Adjust_speed         => -- Only support 1 speed per compile. Not used
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
                if(oob_tx_idle = '1') then -- wait until comreset finished
                    phy_init_next_state <= HP2_HR_Await_COMINIT;
                else
                    phy_init_next_state <= HP1_HR_Reset;
                end if;

            when    HP2_HR_Await_COMINIT         =>
                if(oob_signal_received = COMINIT) then -- check if we receive a cominit
                    phy_init_next_state <= HP2B_HR_Await_No_COMINIT;
                elsif(retry_time_elapsed >= RETRY_INTERVAL and phy_init_state = phy_init_prev_state) then -- if retry's expire, go back to reset
                    phy_init_next_state <= HP1_HR_Reset;
                else
                    phy_init_next_state <= HP2_HR_Await_COMINIT;
                end if;

            when    HP2B_HR_Await_No_COMINIT      => -- wait until link idle
                if(oob_rx_idle = '1') then
                    phy_init_next_state <= HP4_HR_COMWAKE;
                else
                    phy_init_next_state <= HP2B_HR_Await_No_COMINIT;
                end if;

            when    HP3_HR_Calibrate            =>
            -- NOT SUPPORTED FOR NOW, should NEVER end up here since HP2B bypasses this step
                phy_init_next_state <= HP4_HR_COMWAKE;

            when    HP4_HR_COMWAKE              => -- send a comwake
                if(oob_tx_idle = '0') then
                    phy_init_next_state <= HP4_HR_COMWAKE;
                elsif(oob_signal_received = COMWAKE) then -- could receive a comwake while still in this state. May need to skip HP5
                    phy_init_next_state <= HP5B_HR_Await_No_COMWAKE;
                else
                    phy_init_next_state <= HP5_HR_Await_COMWAKE;
                end if;

            when    HP5_HR_Await_COMWAKE         => -- wait to receive comwake
                if(oob_signal_received = COMWAKE) then
                    phy_init_next_state <= HP5B_HR_Await_No_COMWAKE;
                elsif(retry_time_elapsed > RETRY_INTERVAL and phy_init_state = phy_init_prev_state) then -- check if counter expired
                    if(resume_pending = '0') then -- if we were coming from powerdown (not supported!) then we would only go back to comwake
                        phy_init_next_state <= HP1_HR_RESET;
                    else
                        phy_init_next_state <= HP4_HR_COMWAKE;
                    end if;
                else
                    phy_init_next_state <= HP5_HR_Await_COMWAKE;
                end if;

            when    HP5B_HR_Await_No_COMWAKE      => -- wait until link idle before sending d10.2
                if(oob_rx_idle = '1') then
                    phy_init_next_state <= HP6_HR_Await_Align;
                else
                    phy_init_next_state <= HP5B_HR_Await_No_COMWAKE;
                end if;

            when    HP6_HR_Await_Align           => -- wait until we sync up our word aligner
                if(rx_signaldetect = '0') then -- if the link goes dead, try again
                    phy_init_next_state <= HP4_HR_COMWAKE;
                elsif(rx_syncstatus = ALL_WORDS_SYNC and is_byte_ordered = '1' and rx_ordered_data_s = ALIGNp and lock_done = '1' and signal_stable_time > 32) then -- good data (ALIGNp) and byte orderer success, and locks switched
                    phy_init_next_state <= HP7_HR_Send_Align;
                elsif(retry_time_elapsed > ALIGN_INTERVAL and phy_init_state = phy_init_prev_state) then -- counter expires
                    phy_init_next_state <= HP1_HR_RESET;
                else
                    phy_init_next_state <= HP6_HR_Await_Align;
                end if;

            when    HP7_HR_Send_Align            => -- Achieved lock. Tell the device and wait for non-align primitives
                if(rx_signaldetect = '0') then
                    phy_init_next_state <= HP4_HR_COMWAKE;
                elsif(consecutive_non_aligns >= 3) then
                    phy_init_next_state <= HP8_HR_Ready;
                else
                    phy_init_next_state <= HP7_HR_Send_Align;
                end if;

            when    HP8_HR_Ready                => -- ready to transmit
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
end architecture phy_layer_init_arch;
