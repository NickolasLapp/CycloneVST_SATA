library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;

entity OOB_signal_detect is
    port(
        rxclkout         : in  std_logic;
        txclkout         : in  std_logic;
        reset            : in  std_logic;

        rx_data : in  std_logic_vector(31 downto 0);
        rx_signaldetect  : in  std_logic;

        oob_signal_to_send  : in  OOB_SIGNAL;
        oob_rx_idle        : out std_logic;
        oob_tx_idle        : out std_logic;

        oob_signal_received: out OOB_SIGNAL;

        tx_forceelecidle : out std_logic
    );
end entity OOB_signal_detect;

architecture OOB_signal_detect_arch of OOB_signal_detect is


    signal sending_signal : OOB_SIGNAL := NONE;
    signal receiving_signal : OOB_SIGNAL := NONE;

    signal tx_state : OOB_STATE_TYPE := IDLE;
    signal tx_next_state : OOB_STATE_TYPE := IDLE;

    signal rx_signaldetect_prev : std_logic;

    signal UICount_sent : std_logic_vector(15 downto 0);
    signal pauses_sent  : std_logic_vector(7 downto 0);
    signal pulses_sent  : std_logic_vector(7 downto 0);

    signal UICount_recvd : std_logic_vector(15 downto 0);
    signal pauses_recvd  : std_logic_vector(7 downto 0);
    signal pulses_recvd  : std_logic_vector(7 downto 0);

begin
    -- State Register
    process(txclkout, reset)
    begin
        if(reset = '1') then
            tx_state <= IDLE;
        elsif(rising_edge(txclkout)) then
            tx_state <= tx_next_state;
        end if;
    end process;

    -- Count Received Pauses/Signals
    process(rxclkout, reset)
    begin
        if(reset = '1') then
            UICount_recvd <= (others => '0');
            pauses_recvd  <= (others => '0');
            pulses_recvd  <= (others => '0');
        elsif(rising_edge(rxclkout)) then
            rx_signaldetect_prev <= rx_signaldetect;
            if(rx_signaldetect = rx_signaldetect_prev) then
                UICount_recvd <= UICount_recvd + 1;
            else -- detect switch
                UICount_recvd <= (others => '0');
                if(rx_signaldetect = '0') then -- pulse just finished
                    if(UICount_recvd > MIN_DETECT_PULSE_COUNT and UICount_recvd < MAX_DETECT_PULSE_COUNT) then
                        pulses_recvd <= pulses_recvd + 1;
                    else
                        -- error on number of pulses received...
                        receiving_signal <= NONE;
                        pulses_recvd <= (others => '0');
                        pauses_recvd <= (others => '0');
                        UICount_recvd <= (others => '0');
                    end if;
                else -- pause just finished
                    if(UICount_recvd > MIN_COMINIT_DETECT_PAUSE_COUNT and UICount_recvd < MAX_COMINIT_DETECT_PAUSE_COUNT) then
                        if(receiving_signal = NONE or receiving_signal = COMINIT) then
                            pauses_recvd <= pauses_recvd + 1;
                            receiving_signal <= COMINIT;
                        else
                            receiving_signal <= NONE;
                            pulses_recvd <= (others => '0');
                            pauses_recvd <= (others => '0');
                            UICount_recvd <= (others => '0');
                        end if;
                    elsif(UICount_recvd > MIN_COMWAKE_DETECT_PAUSE_COUNT and UICount_recvd < MAX_COMWAKE_DETECT_PAUSE_COUNT) then
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
    process(txclkout, reset)
    begin
        if(reset = '1') then
            UICount_sent  <= (others => '0');
            pauses_sent   <= (others => '0');
            pulses_sent   <= (others => '0');
        elsif(rising_edge(txclkout)) then
            case tx_state is
                when IDLE       =>
                    UICount_sent <= (others => '0');
                    pauses_sent <= (others => '0');
                    pulses_sent <= (others => '0');
                    if(oob_signal_to_send = COMWAKE or oob_signal_to_send = COMRESET) then
                        sending_signal <= oob_signal_to_send;
                    end if;
                    tx_forceelecidle <= '1';

                when SEND_PULSE =>
                    if(UICount_sent = TRANSMIT_PULSE_COUNT) then
                        UICount_sent <= (others =>'0');
                        pulses_sent <= pulses_sent + 1;
                    else
                        UICount_sent <= UICount_sent + 1;
                    end if;
                    tx_forceelecidle <= '0';

                when SEND_PAUSE =>
                    if(UICount_sent = COMWAKE_PAUSE_COUNT and sending_signal = COMWAKE) then
                        UICount_sent <= (others => '0');
                        pauses_sent <= pauses_sent + 1;
                    elsif(UICount_sent = COMRESET_PAUSE_COUNT and sending_signal = COMRESET) then
                        UICount_sent <= (others => '0');
                        pauses_sent <= pauses_sent + 1;
                    else
                        UICount_sent <= UICount_sent + 1;
                    end if;
                    tx_forceelecidle <= '1';

                when others     =>
                    tx_forceelecidle <= '0';
            end case;
        end if;
    end process;



    -- Next State Logic
    process(UICount_sent, pauses_sent, pulses_sent, oob_signal_to_send, tx_state, sending_signal)
    begin
        case(tx_state) is
            when IDLE =>
                if(oob_signal_to_send = COMWAKE or oob_signal_to_send = COMRESET) then
                    tx_next_state <= SEND_PULSE;
                else
                    tx_next_state <= IDLE;
                end if;
            when SEND_PULSE =>
                if (UICount_sent = TRANSMIT_PULSE_COUNT) then
                    tx_next_state <= SEND_PAUSE;
                else
                    tx_next_state <= SEND_PULSE;
                end if;

            when SEND_PAUSE =>
                if(sending_signal = COMWAKE) then
                    if(UICount_sent < COMWAKE_PAUSE_COUNT) then
                        tx_next_state <= SEND_PAUSE;
                    else
                        if(pauses_sent >= NUM_PAUSES_TO_SEND and pulses_sent >= NUM_PULSES_TO_SEND) then
                            tx_next_state <= IDLE;
                        else
                            tx_next_state <= SEND_PULSE;
                        end if;
                    end if;
                else -- sending_signal = COMRESET
                    if(UICount_sent < COMRESET_PAUSE_COUNT) then
                        tx_next_state <= SEND_PAUSE;
                    else
                        if(pauses_sent >= NUM_PAUSES_TO_SEND and pulses_sent >= NUM_PULSES_TO_SEND) then
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

    process(rxclkout, reset)
    begin
        if(reset = '1') then
            oob_signal_received <= NONE;
        elsif(rising_edge(rxclkout)) then
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
    process(rxclkout, reset)
    begin
        if(reset = '1') then
            oob_rx_idle <= '0';
        elsif(rising_edge(rxclkout)) then
            if(UICount_recvd > MIN_IDLE_DETECT_COUNT) then
                oob_rx_idle <= '1';
            else
                oob_rx_idle <= '0';
            end if;
        end if;
    end process;

    -- idle when tx_state = IDLE, and no signal to send coming up
    process(txclkout, reset)
    begin
        if(reset = '1') then
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
