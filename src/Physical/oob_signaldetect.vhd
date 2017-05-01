library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;

entity OOB_signal_detect is
    port(
        rxclkout         : in  std_logic;
        txclkout         : in  std_logic;
        rst_n            : in  std_logic;

        rx_data : in  std_logic_vector(31 downto 0); -- received data from xcvrs
        rx_signaldetect  : in  std_logic; -- rx signaldetect, which indicates when the line is active

        oob_signal_to_send  : in  OOB_SIGNAL; -- what kind of OOB signal should we be sending
        oob_rx_idle        : out std_logic; -- is the OOB reciever idle? (not receiving oob signal from device)
        oob_tx_idle        : out std_logic; -- is the OOB transmitter idle? (not transmitting oob signal to device)

        oob_signal_received: out OOB_SIGNAL; -- what signal did the oob detector see?

        tx_forceelecidle : out std_logic -- used to control when we force the link to idle
    );
end entity OOB_signal_detect;

architecture OOB_signal_detect_arch of OOB_signal_detect is


    signal sending_signal : OOB_SIGNAL := NONE;
    signal receiving_signal : OOB_SIGNAL := NONE;

    signal tx_state : OOB_STATE_TYPE := IDLE; -- are we sending pauses or pulses
    signal tx_next_state : OOB_STATE_TYPE := IDLE;

    signal rx_signaldetect_prev : std_logic; -- need to store the previous value to detect a change in the link condition

    signal UICount_sent : std_logic_vector(15 downto 0); -- how many Unit Intervals have we sent?
    signal pauses_sent  : std_logic_vector(7 downto 0); -- how many pauses have we sent?
    signal pulses_sent  : std_logic_vector(7 downto 0); -- how many pulses have we sent?

    signal UICount_recvd : std_logic_vector(15 downto 0); -- how many Unit Intervals have we recvd?
    signal pauses_recvd  : std_logic_vector(7 downto 0); -- how many pauses have we recvd?
    signal pulses_recvd  : std_logic_vector(7 downto 0); -- how many pulses have we recvd?

begin
    -- State Register
    process(txclkout, rst_n)
    begin
        if(rst_n = '0') then
            tx_state <= IDLE;
        elsif(rising_edge(txclkout)) then
            tx_state <= tx_next_state;
        end if;
    end process;

    -- Count Received Pauses/Signals
    process(rxclkout, rst_n)
    begin
        if(rst_n = '0') then
            UICount_recvd <= (others => '0');
            pauses_recvd  <= (others => '0');
            pulses_recvd  <= (others => '0');
        elsif(rising_edge(rxclkout)) then
            rx_signaldetect_prev <= rx_signaldetect; -- store received signaldetect
            if(rx_signaldetect = rx_signaldetect_prev) then -- if we are still receiving the same state, we increment the UI Count
                UICount_recvd <= UICount_recvd + 1;
            else -- otherwise, there is a switch
                UICount_recvd <= (others => '0');
                if(rx_signaldetect = '0') then -- pulse just finished
                    if(UICount_recvd > MIN_DETECT_PULSE_COUNT and UICount_recvd < MAX_DETECT_PULSE_COUNT) then -- if we are outside of specified data boundaries for pulses recieved, reset the counters and try again
                        pulses_recvd <= pulses_recvd + 1;
                    else
                        -- error on number of pulses received...
                        receiving_signal <= NONE;
                        pulses_recvd <= (others => '0');
                        pauses_recvd <= (others => '0');
                        UICount_recvd <= (others => '0');
                    end if;
                else -- pause just finished
                    if(UICount_recvd > MIN_COMINIT_DETECT_PAUSE_COUNT and UICount_recvd < MAX_COMINIT_DETECT_PAUSE_COUNT) then -- if we are outside of specified data boundaries for pauses recieved, reset the counters and try again
                        if(receiving_signal = NONE or receiving_signal = COMINIT) then
                            pauses_recvd <= pauses_recvd + 1;
                            receiving_signal <= COMINIT;
                        else
                            receiving_signal <= NONE;
                            pulses_recvd <= (others => '0');
                            pauses_recvd <= (others => '0');
                            UICount_recvd <= (others => '0');
                        end if;
                    elsif(UICount_recvd > MIN_COMWAKE_DETECT_PAUSE_COUNT and UICount_recvd < MAX_COMWAKE_DETECT_PAUSE_COUNT) then -- if we are outside of specified data boundaries for pauses recieved, reset the counters and try again
                        if(receiving_signal = NONE or receiving_signal = COMWAKE) then
                            pauses_recvd <= pauses_recvd + 1;
                            receiving_signal <= COMWAKE;
                        else
                            receiving_signal <= NONE;
                            pulses_recvd <= (others => '0');
                            pauses_recvd <= (others => '0');
                            UICount_recvd <= (others => '0');
                        end if;
                    else
                        receiving_signal <= NONE;
                        pulses_recvd <= (others => '0');
                        pauses_recvd <= (others => '0');
                        UICount_recvd <= (others => '0');
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- transmit necessary signals
    process(txclkout, rst_n)
    begin
        if(rst_n = '0') then
            UICount_sent  <= (others => '0');
            pauses_sent   <= (others => '0');
            pulses_sent   <= (others => '0');
        elsif(rising_edge(txclkout)) then
            case tx_state is
                when IDLE       => -- don't send any signals
                    UICount_sent <= (others => '0');
                    pauses_sent <= (others => '0');
                    pulses_sent <= (others => '0');
                    if(oob_signal_to_send = COMWAKE or oob_signal_to_send = COMRESET) then -- if we get a new signal, start to send it on next clock edge
                        sending_signal <= oob_signal_to_send;
                    end if;
                    tx_forceelecidle <= '1'; -- link should be idle if not sending

                when SEND_PULSE => -- pulses are all the same length. Increment the UI count sent, or reset and prepare to send a pause
                    if(UICount_sent = TRANSMIT_PULSE_COUNT) then
                        UICount_sent <= (others =>'0');
                        pulses_sent <= pulses_sent + 1;
                    else
                        UICount_sent <= UICount_sent + 1;
                    end if;
                    tx_forceelecidle <= '0'; -- if we are currently sending a pulse, we should not be idle

                when SEND_PAUSE => -- pauses are not all the same length. Increment the UI count sent, or reset and prepare to send a pulse
                    if(UICount_sent = COMWAKE_PAUSE_COUNT and sending_signal = COMWAKE) then -- reset if we are sending comwake and we have sent the right count
                        UICount_sent <= (others => '0');
                        pauses_sent <= pauses_sent + 1;
                    elsif(UICount_sent = COMRESET_PAUSE_COUNT and sending_signal = COMRESET) then -- reset if we are sending comreset and we have sent the right count
                        UICount_sent <= (others => '0');
                        pauses_sent <= pauses_sent + 1;
                    else
                        UICount_sent <= UICount_sent + 1;
                    end if;
                    tx_forceelecidle <= '1'; -- link must be idle during pause

                when others     =>
                    tx_forceelecidle <= '0';
            end case;
        end if;
    end process;



    -- Next State Logic
    process(UICount_sent, pauses_sent, pulses_sent, oob_signal_to_send, tx_state, sending_signal)
    begin
        case(tx_state) is
            when IDLE => -- in idle we will check the oob signal to send
                if(oob_signal_to_send = COMWAKE or oob_signal_to_send = COMRESET) then
                    tx_next_state <= SEND_PULSE;
                else
                    tx_next_state <= IDLE;
                end if;
            when SEND_PULSE => -- if we are sending a pulse, wait until the count rolls over, then send a pause
                if (UICount_sent = TRANSMIT_PULSE_COUNT) then
                    tx_next_state <= SEND_PAUSE;
                else
                    tx_next_state <= SEND_PULSE;
                end if;

            when SEND_PAUSE => -- if we are sending a pause, check what kind (comwake/reset). If the count rolls over, send a pulse
                if(sending_signal = COMWAKE) then
                    if(UICount_sent < COMWAKE_PAUSE_COUNT) then
                        tx_next_state <= SEND_PAUSE;
                    else
                        if(pauses_sent >= NUM_PAUSES_TO_SEND and pulses_sent >= NUM_PULSES_TO_SEND) then -- if we have finished sending the signal, go to idle
                            tx_next_state <= IDLE;
                        else
                            tx_next_state <= SEND_PULSE;
                        end if;
                    end if;
                else -- sending_signal = COMRESET
                    if(UICount_sent < COMRESET_PAUSE_COUNT) then
                        tx_next_state <= SEND_PAUSE;
                    else
                        if(pauses_sent >= NUM_PAUSES_TO_SEND and pulses_sent >= NUM_PULSES_TO_SEND) then -- if we have finished sending the signal, go to idle
                            tx_next_state <= IDLE;
                        else
                            tx_next_state <= SEND_PULSE;
                        end if;
                    end if;
                end if;
            when others =>
                tx_next_state <= IDLE;
        end case;
    end process;

    process(rxclkout, rst_n)
    begin
        if(rst_n = '0') then
            oob_signal_received <= NONE;
        elsif(rising_edge(rxclkout)) then -- what signal are we currently receiving?
            if(pauses_recvd = NUM_PAUSES_TO_SEND and receiving_signal = COMINIT) then
                oob_signal_received <= COMINIT;
            elsif(pauses_recvd = NUM_PAUSES_TO_SEND and receiving_signal = COMWAKE) then
                oob_signal_received <= COMWAKE;
            else
                oob_signal_received <= NONE;
            end if;
        end if;
    end process;



    -- idle when >525ns have passed, which is 20*40 GEN 1 UI
    process(rxclkout, rst_n)
    begin
        if(rst_n = '0') then
            oob_rx_idle <= '0';
        elsif(rising_edge(rxclkout)) then
            if(UICount_recvd > MIN_IDLE_DETECT_COUNT) then
                oob_rx_idle <= '1'; -- we have not received a signal in the idle time, so we are rx idle
            else
                oob_rx_idle <= '0';
            end if;
        end if;
    end process;

    -- idle when tx_state = IDLE, and no signal to send coming up
    process(txclkout, rst_n)
    begin
        if(rst_n = '0') then
            oob_tx_idle <= '0';
        elsif(rising_edge(txclkout)) then
            if(tx_state = IDLE and oob_signal_to_send = NONE) then
                oob_tx_idle <= '1';
            else
                oob_tx_idle <= '0';
            end if;
        end if;
    end process;

end architecture OOB_signal_detect_arch;
